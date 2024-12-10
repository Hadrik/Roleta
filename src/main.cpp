#include "config.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#if OTA == 1
  #include <ESP8266mDNS.h>
  #include <WiFiUdp.h>
  #include <ArduinoOTA.h>
#endif
#include <OneButton.h>
#include <AsyncMqttClient.h>
#include <Ticker.h>
#include "Led.h"
#include "Motor.h"
#include "Auto.h"
#include "DebugLed.h"
#if DEBUG == 1
  #include <SoftwareSerial.h>
  SoftwareSerial s;
#endif

using namespace config;

// MQTT
AsyncMqttClient client;
Ticker mqtt_reconnect_timer;

// WiFi
WiFiEventHandler wifi_connect_handler;
WiFiEventHandler wifi_disconnect_handler;
Ticker wifi_reconnect_timer;

// OTA
#if OTA == 1
  WiFiServer telnet_server(8266);
#endif

Motor motor(&Serial,
            stepper::R_SENSE,
            stepper::EN_PIN,
            stepper::DIR_PIN,
            stepper::STEP_PIN,
            stepper::HOME_PIN,
            stepper::TOTAL_STEPS
            #ifdef DEBUG
            ,&s
            #endif
            );
Ticker motor_update;
Led led(LED_PIN);
Auto light_control(photoresistor::PIN, photoresistor::ENABLE);
OneButton btn_down(button_down::BTN_PIN, button_down::ACTIVE_LOW);
OneButton btn_up(button_up::BTN_PIN, button_up::ACTIVE_LOW);
DebugLed debug_led(DEBUG_LED_PIN, &s, true);
double target_up = 0.1;
double target_down = 0.9;

void updateState() {
  debug("Sending full update!\n");

  client.publish((mqtt::TOPIC_STAT + "/exact").c_str(), mqtt::QOS, mqtt::RETAIN, std::to_string(motor.getTargetP()).c_str());
  client.publish((mqtt::TOPIC_STAT + "/saved/up").c_str(), mqtt::QOS, mqtt::RETAIN, std::to_string(target_up).c_str());
  client.publish((mqtt::TOPIC_STAT + "/saved/down").c_str(), mqtt::QOS, mqtt::RETAIN, std::to_string(target_down).c_str());
  client.publish((mqtt::TOPIC_STAT + "/steps").c_str(), mqtt::QOS, mqtt::RETAIN, std::to_string(motor.getTotalSteps()).c_str());
  client.publish((mqtt::TOPIC_STAT + "/accel").c_str(), mqtt::QOS, mqtt::RETAIN, std::to_string(motor.getAcceleration()).c_str());
  client.publish((mqtt::TOPIC_STAT + "/accel").c_str(), mqtt::QOS, mqtt::RETAIN, std::to_string(motor.getSpeed()).c_str());
  client.publish((mqtt::TOPIC_STAT + "/light").c_str(), mqtt::QOS, mqtt::RETAIN, std::to_string(light_control.read()).c_str());
  client.publish((mqtt::TOPIC_STAT + "/auto/on").c_str(), mqtt::QOS, mqtt::RETAIN, std::to_string(light_control.enabled()).c_str());
  client.publish((mqtt::TOPIC_STAT + "/auto/day").c_str(), mqtt::QOS, mqtt::RETAIN, std::to_string(light_control.getDayThreshold()).c_str());
  client.publish((mqtt::TOPIC_STAT + "/auto/night").c_str(), mqtt::QOS, mqtt::RETAIN, std::to_string(light_control.getNightThreshold()).c_str());
}

void singleU() {
  led.onFor(500);
  motor.setTargetP(target_up);
}

void singleD() {
  led.onFor(500);
  motor.setTargetP(target_down);
}

void doubleU() {
  led.onFor(2000);
  target_up = motor.getPositionP();
}

void doubleD() {
  led.onFor(2000);
  target_down = motor.getPositionP();
}

void longUp() {
  led.on();
  motor.setTargetP(0.0);
}

void longDown() {
  led.on();
  motor.setTargetP(1.0);
}

void longStop() {
  led.off();
  motor.setTarget(motor.getPosition());
}

void onMessage(char* _topic, char* payload, AsyncMqttClientMessageProperties properties, size_t length, size_t index, size_t total) {
  std::string topic(_topic);
  debug("Recieved message on topic: %s\n", topic.c_str());
  if (topic == mqtt::TOPIC_SET) {
    debug("Invalid topic\n");
    return;
  }
  std::string command = topic.substr(mqtt::TOPIC_SET.length() + 1);
  debug("Command: %s\n", command.c_str());

  payload[length] = '\0';
  std::string msg(payload);
  debug("Message: %s\n", msg.c_str());

  if (command == "position") {
    if (msg == "up") {
      motor.setTargetP(target_up);
    } else if (msg == "down") {
      motor.setTargetP(target_down);
    }
    client.publish((mqtt::TOPIC_STAT + "/exact").c_str(), mqtt::QOS, mqtt::RETAIN, std::to_string(motor.getTargetP()).c_str());
  } else if (command == "exact") {
    motor.setTargetP(std::stod(msg));
    client.publish((mqtt::TOPIC_STAT + "/exact").c_str(), mqtt::QOS, mqtt::RETAIN, std::to_string(motor.getTargetP()).c_str());
  } else if (command == "save") {
    if (msg == "up") {
      target_up = motor.getTargetP();
      client.publish((mqtt::TOPIC_STAT + "/saved/up").c_str(), mqtt::QOS, mqtt::RETAIN, std::to_string(target_up).c_str());
    } else if (msg == "down") {
      target_down = motor.getTargetP();
      client.publish((mqtt::TOPIC_STAT + "/saved/down").c_str(), mqtt::QOS, mqtt::RETAIN, std::to_string(target_down).c_str());
    }
  } else if (command == "steps") {
    motor.setTotalSteps(std::stoul(msg));
    client.publish((mqtt::TOPIC_STAT + "/steps").c_str(), mqtt::QOS, mqtt::RETAIN, std::to_string(motor.getTotalSteps()).c_str());
  } else if (command == "accel") {
    motor.setAcceleration(std::stof(msg));
    client.publish((mqtt::TOPIC_STAT + "/accel").c_str(), mqtt::QOS, mqtt::RETAIN, std::to_string(motor.getAcceleration()).c_str());
  } else if (command == "speed") {
    motor.setSpeed(std::stof(msg));
    client.publish((mqtt::TOPIC_STAT + "/accel").c_str(), mqtt::QOS, mqtt::RETAIN, std::to_string(motor.getSpeed()).c_str());
  } else if (command == "auto/on") {
    light_control.enabled(msg == "1");
    client.publish((mqtt::TOPIC_STAT + "/auto/on").c_str(), mqtt::QOS, mqtt::RETAIN, std::to_string(light_control.enabled()).c_str());
  } else if (command == "auto/day") {
    light_control.setDayThreshold(std::stod(msg));
    client.publish((mqtt::TOPIC_STAT + "/auto/day").c_str(), mqtt::QOS, mqtt::RETAIN, std::to_string(light_control.getDayThreshold()).c_str());
  } else if (command == "auto/night") {
    light_control.setNightThreshold(std::stod(msg));
    client.publish((mqtt::TOPIC_STAT + "/auto/night").c_str(), mqtt::QOS, mqtt::RETAIN, std::to_string(light_control.getNightThreshold()).c_str());
  }
}

void IRAM_ATTR motorLoop() {
  motor.update();
}

void setup() {
  #ifdef DEBUG
  s.begin(9600, SWSERIAL_8N1, -1, 1);
  s.enableRx(false);
  #endif
  debug("\nStarting!\n");

  // Debug LED
  debug_led.addState("wifi", 1500, 1, true);
  debug_led.addState("mqtt", 500, 0, true);
  debug("Debug LED setup finished\n");

  // WiFi
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_STA);
  WiFi.persistent(true);
  wifi_connect_handler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& event) {
    debug("Wifi connected\n");
    debug_led.setState("wifi", false);
    wifi_reconnect_timer.detach();
    telnet_server.begin();
    ArduinoOTA.begin();
    client.connect();
  });
  wifi_disconnect_handler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& event) {
    debug("Wifi disconnected, reason: %d ; SSID: %s ; status: %d\n", (int)event.reason, event.ssid.c_str(), (int)WiFi.status());
    debug_led.setState("wifi", true);
    mqtt_reconnect_timer.detach();
    telnet_server.stop();
    ArduinoOTA.end();
    wifi_reconnect_timer.once(wifi::RECONNECT, []() {
      debug("Wifi reconnecting - \n");
      if (WiFi.reconnect()) {
        debug("success\n");
      } else {
        debug("fail\n");
      }
    });
  });
  debug("Wifi setup finished\n");

  // MQTT
  client.onConnect([](bool session) {
    debug("MQTT connected\n");
    debug_led.setState("mqtt", false);
    const char* topic = (mqtt::TOPIC_SET + "/#").c_str();
    client.subscribe(topic, mqtt::QOS);
    debug("Subscribed to %s\n", topic);
    updateState();
  });
  client.onDisconnect([](AsyncMqttClientDisconnectReason reason) {
    debug("MQTT disconnected, reason: %d\n", (int)reason);
    debug_led.setState("mqtt", true);
    if (WiFi.isConnected()) {
      debug("Will try to reconnect in %d seconds\n", mqtt::RECONNECT);
      mqtt_reconnect_timer.once(mqtt::RECONNECT, []() {
        debug("MQTT reconnecting\n");
        client.connect();
      });
    } else {
      debug("Cannot reconnect MQTT - Wifi not connected\n");
    }
  });
  client.onMessage(onMessage);
  client.setClientId(mqtt::CLIENT_ID.c_str());
  client.setCredentials(mqtt::USERNAME.c_str(), mqtt::PASSWORD.c_str());
  client.setServer(mqtt::SERVER.c_str(), mqtt::PORT);

  WiFi.begin(wifi::SSID.c_str(), wifi::PASSWORD.c_str());

  // Buttons
  btn_down.attachClick(singleD);
  btn_down.attachDoubleClick(doubleD);
  btn_down.attachLongPressStart(longDown);
  btn_down.attachLongPressStop(longStop);
  btn_up.attachClick(singleU);
  btn_up.attachDoubleClick(doubleU);
  btn_up.attachLongPressStart(longUp);
  btn_up.attachLongPressStop(longStop);
  debug("Button setup finished\n");

  // Motor
  motor.begin();
  motor.setAcceleration(stepper::ACCELERATION).setSpeed(stepper::SPEED);
  motor_update.attach_ms(10, motorLoop);
  debug("Motor setup finished\n");

  // OTA
  #if OTA == 1
  ArduinoOTA.onStart([]() {
    motor.estop();
    debug("OTA started\n");
  });
  ArduinoOTA.onEnd([]() {
    debug("OTA finished\n");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    debug("OTA error: %d\n", error);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    debug("OTA progress: %u%%\n", (progress / (total / 100)));
  });
  ArduinoOTA.setHostname(mqtt::CLIENT_ID.c_str());
  debug("OTA setup finished\n");
  #endif

  debug("End of setup!\n");
}

void loop() {
  btn_down.tick();
  btn_up.tick();
  led.update();
  light_control.update();

  #if OTA == 1
  ArduinoOTA.handle();
  #endif
}