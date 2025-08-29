#include "robot.h"

String serString1;

/**
 * sendMessage creates a string of the form
 *      topic:message
 * which is what the corresponding ESP32 code expects.
 * */
void Robot::sendMessage(const String& topic, const String& message)
{
    static uint32_t lastSend = 0;
    uint32_t currTime = millis();
    uint16_t interval = 500;

    if(currTime - lastSend >= interval) {
        lastSend = currTime;
        Serial1.println(topic + String(':') + message);
    }
}

bool Robot::checkSerial1(void)
{
    while(Serial1.available())
    {
        char c = Serial1.read();
        serString1 += c;

        if(c == '\n')
        {
            return true;
        }
    }

    return false;
}

/**
 * This basic example sends the time (from millis()) every
 * five seconds. See the `readme.md` in the root directory of this repo for 
 * how to set up the WiFi. 
 * */
void Robot::wifi_loop() 
{

    // Check to see if we've received anything
    if(checkSerial1())
    {
        Serial.print("Rec'd:\t");
        Serial.print(serString1);
        serString1 = "";
    }

    sendMessage("state", rs_to_string(robotState));
    sendMessage("mode", rcm_to_string(robotCtrlMode));
}