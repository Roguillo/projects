/**
 * robot-nav.cpp is where you should put navigation routines.
 */

#include "robot.h"
#include "chassis.h"
#include "event_timer.h"

// Store info from previous position
float prevPoseX = 0; 
float prevPoseY = 0;
float prevTheta = 0;

void Robot::UpdatePose(const Twist& twist)
{
    /**
     * TODO: Add your FK algorithm to update currPose here.
     */

    // speeds in cm/s
    float speedRight = twist.u + twist.omega * ROBOT_RADIUS;
    float speedLeft = twist.u - twist.omega * ROBOT_RADIUS;

    // calculate distance from control loop time and speeds
    // control loop is 20 ms
    float distRight = speedRight * 20 / 1000;
    float distLeft = speedLeft * 20 / 1000;

    // calculate thetas
    currPose.theta = prevTheta + (distRight - distLeft) / (ROBOT_RADIUS * 2);
    float theta_2 = (prevTheta + currPose.theta) / 2;

    currPose.x = prevPoseX + (distLeft + distRight) / 2 * cos(theta_2);
    currPose.y = prevPoseY + (distLeft + distRight) / 2 * sin(theta_2);

    prevPoseX = currPose.x;
    prevPoseY = currPose.y;
    prevTheta = currPose.theta;

#ifdef __NAV_DEBUG__
    TeleplotPrint("x", currPose.x);
    TeleplotPrint("y", currPose.y);
    TeleplotPrint("theta", currPose.theta);
#endif

}

/**
 * Sets a destination in the lab frame.
 */
void Robot::SetDestination(const Pose& dest)
{
    /**
     * TODO: Turn on LED, as well.
     */

    digitalWrite(13, 1);

    Serial.print("Setting dest to: ");
    Serial.print(dest.x);
    Serial.print(", ");
    Serial.print(dest.y);
    Serial.print('\n');

    destPose = dest;
    robotState = ROBOT_DRIVE_TO_POINT;
}

bool Robot::CheckReachedDestination(void)
{
    bool retVal = false;
    /**
     * TODO: Add code to check if you've reached destination here.
     */

    // min distance the robot must be within to count as arrived to a point
    float distanceBuffer = 5;

    float dist_x = abs(destPose.x - currPose.x);
    float dist_y = abs(destPose.y - currPose.y);

    // check if robot is withing acceptible distance
    if(dist_x <= distanceBuffer && dist_y <= distanceBuffer)
    {
        retVal = true;
    }

    return retVal;
}

void Robot::DriveToPoint(void)
{
    if(robotState == ROBOT_DRIVE_TO_POINT)
    {
        /**
         * TODO: Add your IK algorithm here. 
         */

        // find errors for all coordinates and angle
        float error_x = destPose.x - currPose.x;
        float error_y = destPose.y - currPose.y;
        float error_distance = sqrt(pow(error_x,2) + pow(error_y,2));

        float distance_cap = 100;
        if (error_distance > distance_cap) error_distance = distance_cap;

        float target_angle = atan2(error_y, error_x);

        float error_angle = target_angle - currPose.theta;
        float error_pose = 0;//to implement

        if (error_angle > PI) error_angle -= 2*PI;
        if (error_angle < PI) error_angle += 2*PI;
        
        // minimum speed robot will go
        int baseSpeed = 3;

        // weight for angle and dist diffs
        float gainAngle = 5;
        float gainDist = 0.2;
        float gainPose = 0; // to implement

        // proportional control for the robot
        float wheelSpeedRight = gainDist * error_distance + gainAngle * error_angle + gainPose * error_pose ;
        float wheelSpeedLeft = gainDist * error_distance - gainAngle * error_angle - gainPose * error_pose;

        // put values in a twist object to pass to SetTwist()
        Twist updatedTwist;
        updatedTwist.u = (wheelSpeedRight + wheelSpeedLeft) / 2;
        updatedTwist.v = 0;
        updatedTwist.omega = (wheelSpeedRight + wheelSpeedLeft) / (ROBOT_RADIUS * 2);

#ifdef __NAV_DEBUG__
        // Print useful stuff here.
        TeleplotPrint("X Error", error_x);
        TeleplotPrint("Y Error", error_y);
        TeleplotPrint("Distance Error", error_distance);
        TeleplotPrint("Theta Error", error_angle);

        TeleplotPrint("Target Angle", target_angle);
        TeleplotPrint("Right Wheel Speed", wheelSpeedRight);
        TeleplotPrint("Left Wheel Speed", wheelSpeedLeft);
#endif

        /**
         * TODO: Call chassis.SetTwist() to command the motion, based on your calculations above.
         */
        //chassis.SetTwist(updatedTwist);
        chassis.SetWheelSpeeds(wheelSpeedLeft,wheelSpeedRight);

    }
}

void Robot::HandleDestination(void)
{
    /**
     * TODO: Stop and change state. Turn off LED.
     */

    // LED off
    digitalWrite(13, 0);
    EnterSearchingState();
}

void Robot::HandleAprilTag(const AprilTagDatum& tag)
{
    //You may want to comment some of these out when you’re done testing.
    Serial.print("Tag: "); Serial.print(tag.id); Serial.print('\t');
    Serial.print("cx: "); Serial.print(tag.cx); Serial.print('\t');
    Serial.print("cy: ");Serial.print(tag.cy); Serial.print('\t');
    Serial.print("h: ");Serial.print(tag.h); Serial.print('\t');
    Serial.print("w: ");Serial.print(tag.w); Serial.print('\t');
    Serial.print("rot: ");Serial.println(tag.rot); //rotated angle; try turning the tag

    /** TODO: Add code to handle a tag in APPROACHING and SEARCHING states. */
    if(robotState == ROBOT_SEARCHING || robotState == ROBOT_DRIVE_TO_POINT)
    {
        EnterApproachingState();
    }
    else if(robotState == ROBOT_APPROACHING)
    {
        //adjust the motors to align
        int distanceTolerance = 20; //cm
        int headingTolerance = 20; //deg
        if(CheckApproachComplete(headingTolerance, distanceTolerance)) EnterIdleState();
    }
}
void Robot::EnterSearchingState(void)
{
    /** TODO: Set Romi to slowly spin to look for tags. */
    robotState = ROBOT_SEARCHING;
    digitalWrite(LED_BUILTIN, 1);
    chassis.SetTwist(Twist(0, 0, 1.26));
}

void Robot::EnterApproachingState(void)
{
    /**
    * TODO: Turn on the LED when the Romi finds a tag. For extra points,
    * blink out a number that corresponds to the tag ID (must be non-blocking!).
    * Be sure to add code (elsewhere) to turn the LED off when the Romi is
    * done aligning.
    */

    // set state
    robotState = ROBOT_APPROACHING;

    // LED on
    digitalWrite(13,  1);
}

/** Note that the tolerances are in integers, since the camera works
* in integer pixels. If you have another method for calculations,
* you may need floats.
*/
bool Robot::CheckApproachComplete(int headingTolerance, int distanceTolerance)
{
    /** TODO: Add code to determine if the robot is at the correct location. */

    uint16_t targetHeight = 75; // in pixel coords
    uint16_t distance = aprilTag.h - targetHeight;

    uint16_t targetCX = 80;
    uint16_t angle = aprilTag.cx - targetCX;

    if((abs(distance) < distanceTolerance) && (abs(angle) < headingTolerance)) 
    {
        digitalWrite(13, 0);
        return(true); 
    }

    return(false);
}

EventTimer tagLost;

void Robot::ApproachTag(AprilTagDatum &tag)
{

    // reset timer
    long timerDur = 2000;
    tagLost.Start(timerDur);

    // get tag corners (WRONG)
    uint16_t bottom_left[2] = {tag.cx - (tag.w/2), tag.cy - (tag.h/2)};
    uint16_t bottom_right[2] = {tag.cx + (tag.w/2), tag.cy - (tag.h/2)};
    uint16_t top_left[2] = {tag.cx - (tag.w/2), tag.cy + (tag.h/2)};
    uint16_t top_right[2] = {tag.cx + (tag.w/2), tag.cy + (tag.h/2)};
    
    float targetHeight = 100;

    //when tag h is small robot should reverse bc camera on back
    float distance = tag.h - targetHeight;
    
    // bottom right y - bottom left y = negative turn 
    //float angle = bottom_right[1] - bottom_left[1];

    float targetCX = 80;
    // when tag cx is small robot should turn cw (negative) bc camera on back
    float angle = tag.cx - targetCX;

    float distGain = 0.1;
    float anglularGain = 0.05;

    //controller
    float distEffort = -distGain * distance;
    float angleEffort = -anglularGain * angle;

    // combine
    chassis.SetTwist(Twist(distEffort, 0, angleEffort));
}   

bool Robot::CheckTagLost()
{
    if (tagLost.CheckExpired()) return true;
    return false;
}