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
    };
    ROBOT_STATE robotState = ROBOT_IDLE;

    String rs_to_string(int enum_val) {
        switch(enum_val) {
            case 0:
                return("ROBOT_IDLE");
                break;
            case 1:
                return("ROBOT_DRIVE_TO_POINT");
                break;
            case 2:
                return("ROBOT_SEARCHING");
                break;  
            case 3:
                return("ROBOT_APPROACHING");
                break;
            default:
                return("Huh?");
                break;
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
    void HandleAprilTag(const AprilTagDatum& tag);
    void EnterSearchingState(void);
    void EnterApproachingState(void);
    void ApproachTag(AprilTagDatum &tag);
    bool CheckApproachComplete(int headingTolerance, int distanceTolerance);
    bool CheckTagLost(void);
};