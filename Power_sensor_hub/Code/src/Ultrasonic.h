#pragma once
#include <stdint.h>

class Ultrasonic {
    private:
        const uint8_t TRIG_PIN;
        const uint8_t ECHO_PIN; 

    public :
        Ultrasonic(uint8_t trigPin,   uint8_t echoPin) :
                  TRIG_PIN(trigPin), ECHO_PIN(echoPin) {}

        void init();

        void task() { for(;;); }
};