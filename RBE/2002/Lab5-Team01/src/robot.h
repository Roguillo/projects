#pragma once

#include "chassis.h"
#include <openmv.h>

class Robot
{
protected:
    /**
     * We define some modes for you. SETUP is used for adjusting gains and so forth. Most
     * of the activities will run in AUTO. You shouldn't need to mess with these.
     */
    enum ROBOT_CTRL_MODE
    {
        CTRL_TELEOP,
        CTRL_AUTO,
    };
    ROBOT_CTRL_MODE robotCtrlMode = CTRL_AUTO;

    String rcm_to_string(int enum_val) {
        switch(enum_val) {
            case 0: return("CTRL_TELEOP");
            case 1: return("CTRL_AUTO");

            default: return("Huh?");
        }
    }

    /**
     * robotState is used to track the current task of the robot. You will add new states as 
     * the term progresses.
     */
    enum ROBOT_STATE 
    {
        ROBOT_IDLE,
        ROBOT_DRIVE_TO_POINT,
        ROBOT_SEARCHING,
        ROBOT_APPROACHING,
        ITS_RAMPIN_TIME
    };
    ROBOT_STATE robotState = ROBOT_IDLE;

    String rs_to_string(int enum_val) {
        switch(enum_val) {
            case 0: return("ROBOT_IDLE");
            case 1: return("ROBOT_DRIVE_TO_POINT");
            case 2: return("ROBOT_SEARCHING");
            case 3: return("ROBOT_APPROACHING");
            case 4: return("ITS_RAMPIN_TIME");

            default: return("Huh?");
        }
    }

    /* Define the chassis*/
    Chassis chassis;

    // For managing key presses
    String keyString;

    /**
     * For tracking current pose and the destination.
     */
    Pose currPose;
    Pose destPose;

    // Create a camera object
    OpenMV openmv;
    AprilTagDatum aprilTag;

    //Ramp doodad
    bool ramping = true;
    
public:
    Robot(void) {keyString.reserve(10);}
    void InitializeRobot(void);
    void RobotLoop(void);

protected:
    /* For managing IR remote key presses*/
    void HandleKeyCode(int16_t keyCode);

    /* State changes */    
    void EnterIdleState(void);

    /* Mode changes */
    void EnterTeleopMode(void);
    void EnterAutoMode(void);

    // /* Navigation methods.*/
    void UpdatePose(const Twist& u);
    void SetDestination(const Pose& destination);
    void DriveToPoint(void);
    bool CheckReachedDestination(void);
    void HandleDestination(void);

    //CV
    void HandleAprilTag(const AprilTagDatum& tag);
    void EnterSearchingState(void);
    void EnterApproachingState(void);
    void ApproachTag(AprilTagDatum &tag);
    bool CheckApproachComplete(int headingTolerance, int distanceTolerance);
    bool CheckTagLost(void);

    //wireless
    void sendMessage(const String& topic, const String& message);
    bool checkSerial1(void);
    void wireless_init(void);
    void wifi_loop(void);

    //Ramp shenanigans
    void BeginRampDriving(void);
    void ToggleRampMode(void);

    //Emu
    void HandlePitchAngle(float);
};