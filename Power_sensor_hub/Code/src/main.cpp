#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>

// module libraries
#include "IMU.h"
#include "Ultrasonic.h"
#include "LEDs.h"



// peripheral declarations
IMU        imu      (11, 0x68);
LEDs       led_array( 5,   24);
Ultrasonic ultra    ( 8,    9);

// call tasks
void task_IMU       (void* pvParameters) { static_cast<IMU*       >(pvParameters)->task(); }
void task_ultrasonic(void* pvParameters) { static_cast<LEDs*      >(pvParameters)->task(); }
void task_LEDs      (void* pvParameters) { static_cast<Ultrasonic*>(pvParameters)->task(); }

// set up tasks and start scheduler
void setup() {
    imu      .init();
    led_array.init();
    ultra    .init();

    xTaskCreate(task_IMU       , "IMU task"              , 75, &imu      , 2, NULL);
    xTaskCreate(task_ultrasonic, "ultrasonic sensor task", 75, &led_array, 2, NULL);
    xTaskCreate(task_LEDs      , "RGB LED task"          , 75, &ultra    , 2, NULL);

    vTaskStartScheduler();
}

// mandatory for Arduino
void loop();