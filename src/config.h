#include <inttypes.h>
#include <string>
#include <Arduino.h>

#define DEBUG 1
#define OTA 1

// #define LED_DEBUG_DISABLE
// #define MOTOR_DEBUG_DISABLE

namespace config {
  const uint8_t DEBUG_LED_PIN = LED_BUILTIN_AUX; // GPIO16

  namespace stepper {
    // TX = D8 (GPIO15)
    // RX = D7 (GPIO13)
    const uint8_t  EN_PIN       = D1; // GPIO5
    const uint8_t  DIR_PIN      = D3; // GPIO0
    const uint8_t  STEP_PIN     = 3;  // GPIO3 (RXD0)
    const float    R_SENSE      = 0.11f;
    const uint8_t  HOME_PIN     = 255; /* ```255``` for unused */
    const uint32_t TOTAL_STEPS  = 500;
    const float    SPEED        = 1000;
    const float    ACCELERATION = 5000;
  }

  /**
   * Single press = move to target position
   * Double press = set target position
   * Long press   = move untill released
  */
  namespace button_down {
    const uint8_t BTN_PIN = D5; // GPIO14
    const bool    ACTIVE_LOW = true;
  }
  namespace button_up {
    const uint8_t BTN_PIN = D6; // GPIO12
    const bool    ACTIVE_LOW = true;
  }
  namespace photoresistor {
    const bool    ENABLE = false;
    const uint8_t PIN = A0; // ADC0
  }
  const uint8_t LED_PIN = D2; // GPIO4
  
  
  namespace wifi {
    const std::string SSID       = "TurboWifi.2";
    const std::string PASSWORD   = "HovnoKleslo";
    const size_t      RECONNECT  = 10; // time between reconnection attempts (in seconds)
  }

  namespace mqtt {
    const std::string SERVER     = "homeassistant.local";
    const uint16_t    PORT       = 1883;
    const std::string CLIENT_ID  = "roleta";
    const std::string USERNAME   = "mqtt-user";
    const std::string PASSWORD   = "mqtt";
    const uint8_t     QOS        = 1;
    const bool        RETAIN     = false;
    const size_t      RECONNECT  = 30; // time between reconnection attempts (in seconds)

    /**
     * Set data:                HOAS -> ESP
     * @param /position: ```"up" or "down"```   move to saved position
     * @param /exact:    ```double```           move to exact position
     * @param /save:     ```"up" or "down"```   save current position
     * @param /steps:    ```uint32```           set total steps
     * @param /accel:    ```float```            set motor acceleration (steps/sec^2)
     * @param /speed:    ```float```            set max speed (steps/sec)
     * @param /auto/on:  ```bool (0 or 1)```    enable automatic mode
     * @param /auto/day: ```double```           set day threshold
     * @param /auto/night: ```double```         set night threshold
    */
    const std::string TOPIC_SET  = CLIENT_ID + "/set";

    /**
     * State data:               ESP -> HOAS
     * @param /exact:     ```double```          current position
     * @param /saved/up   ```double```          saved 'up' position
     * @param /saved/down ```double```          saved 'down' position
     * @param /steps:     ```uint32```          total steps
     * @param /accel:     ```float```           motor acceleration (steps/sec^2)
     * @param /speed:     ```float```           max speed (steps/sec)
     * @param /light:     ```double```          light level
     * @param /auto/on:   ```bool (0 or 1)```   automatic mode
     * @param /auto/day:  ```double```          day threshold
     * @param /auto/night:```double```          night threshold
    */
    const std::string TOPIC_STAT = CLIENT_ID + "/state";
    const float       STAT_RATE  = 60.0f; // seconds
  }
  
}

#if DEBUG == 1
  #define debug(...) s.printf(__VA_ARGS__)
#else
  #define debug(...)
#endif
