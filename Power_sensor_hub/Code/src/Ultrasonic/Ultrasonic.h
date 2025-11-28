#pragma once

#include <stdint.h>



class Ultrasonic {
    private:
        const uint8_t TRIG_PIN = 8;
        const uint8_t ECHO_PIN = 9; 

    public :
        void task_ultra(void *pvParameters);
};