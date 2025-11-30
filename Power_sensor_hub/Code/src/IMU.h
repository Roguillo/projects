#pragma once
#include <stdint.h>

class IMU {
    private:
        const uint8_t POLL_PIN; 
        const uint8_t I2C_ADDX;

    public :
        IMU(uint8_t pollPin ,  uint8_t I2CAddx) :
           POLL_PIN(pollPin), I2C_ADDX(I2CAddx) {}

        void init();

        void task() { for(;;); }
};