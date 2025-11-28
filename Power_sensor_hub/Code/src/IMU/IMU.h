#pragma once

#include <stdint.h>



class IMU {
    private:
        const uint8_t POLL_PIN =   11; 
        const uint8_t I2C_ADDX = 0x68;

    public :
        IMU() {}
            
        static void task_IMU(void *pvParameters);
};