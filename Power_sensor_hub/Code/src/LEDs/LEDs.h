#pragma once

#include <stdint.h>



class LEDs {
    private:
        const uint8_t LED_PIN    = 5;    
        const uint8_t NUM_LEDS   = 24;
        const uint8_t BRIGHTNESS = 10;

    public :
        void task_LEDs(void *pvParameters);
};