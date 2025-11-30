#pragma once
#include <stdint.h>

class LEDs {
    private:
        const uint8_t LED_PIN;    
        const uint8_t NUM_LEDS;
              uint8_t BRIGHTNESS = 10;

    public :
        LEDs(uint8_t ledPin ,  uint8_t numLeds) :
             LED_PIN(ledPin), NUM_LEDS(numLeds) {}

        void init();

        void task() { for(;;); }
};