#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>

// module libraries
#include "IMU/IMU.h"
#include "Ultrasonic/Ultrasonic.h"
#include "LEDs/LEDs.h"



void setup() {
    xTaskCreate(task_IMU       , "IMU task"              , 75, NULL, 2, NULL);
    xTaskCreate(task_ultrasonic, "ultrasonic sensor task", 75, NULL, 2, NULL);
    xTaskCreate(task_LEDs      , "RGB LED task"          , 75, NULL, 2, NULL);

    vTaskStartScheduler();
}

void loop();