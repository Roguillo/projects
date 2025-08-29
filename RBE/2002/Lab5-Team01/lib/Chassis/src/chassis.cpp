#include "chassis.h"
#include <LSM6.h>
#include "Romi32U4MotorTemplate.h"

Romi32U4EncodedMotor<LEFT_XOR, LEFT_B, PWM_L, DIR_L, OCR_L> leftMotor("L");
Romi32U4EncodedMotor<RIGHT_XOR, RIGHT_B, PWM_R, DIR_R, OCR_R> rightMotor("R");

/**
 * Because it's declared static, we initialize Chassis::loopFlag here.
 */
uint8_t Chassis::loopFlag = 0;

/* pitch angle */
/////accelerometer
//windowed avg arrays for estimated pitch angle
int32_t acc_x_readings[5] = {0, 0, 0, 0, 0};
int32_t acc_z_readings[5] = {0, 0, 0, 0, 0};
//acc avg variables
float acc_x_avg = 0;
float acc_z_avg = 0;
//full 360 deg pitch angle variable
float acc_angle = 0;
/////gyro
int32_t gyro_y_readings[5] = {0, 0, 0, 0, 0};
float gyro_y_avg = 0; // Current gyro average reading
float gyro_y_prev = 0; // Previous gyro reading
float gyro_y_angle = 0; //pitch angle
/////
uint8_t kappa = 0.25; //variable for setting weights in pitch angle weighted average

/**
 * For taking snapshots and raising the flag.
 */
void Chassis::Timer4OverflowISRHandler(void) 
{
    loopFlag++;

    leftMotor.speed = leftMotor.CalcEncoderDelta();
    rightMotor.speed = rightMotor.CalcEncoderDelta();
}

/**
 * ISR for timing. On Timer4 overflow, we take a 'snapshot' of the encoder counts 
 * and raise a flag to let the program it is time to execute the PID calculations.
 */
ISR(TIMER4_OVF_vect)
{
   Chassis::Timer4OverflowISRHandler();
}

/**
 * Sets up a hardware timer on Timer4 to manage motor control on a precise schedule.
 * 
 * We set the timer to set an interrupt flag on overflow, which is handled
 * by ISR(TIMER4_OVF_vect) below.
 */
void Chassis::InitializeMotorControlTimer(void)
{
    Serial.println("InitTimer");
    // Disable interupts while we mess with the Timer4 registers
    cli(); 
  
    // Set up Timer4
    TCCR4A = 0x00; // Disable output to pins
    TCCR4B = 0x0A; // Sets the prescaler -- see pp. 167-8 in datasheet
    TCCR4C = 0x00; // Disables output to pins (but see below for buzzer)
    TCCR4D = 0x00; // Normal mode: count up and roll-over

    /**
     * Calculate TOP based on prescaler and loop duration. Note that loop is in integer ms --
     * there may be some rounding. Multiples of 4 ms will be exact.
     */
    uint16_t top = ((CONTROL_LOOP_PERIOD_MS * 16000ul) >> 9) - 1; // divides by 512

    /**
     * Here we do a little trick to allow full 10-bit register access. 
     * We have 2 _bits_ in TC4H that we can use to add capacity to TOP.
     * 
     * Note that the maximum period is limited by TOP = 0x3FF. If you want
     * a longer period, you'll need to adjust the pre-scaler.
     * 
     * There is no minumum period, but precision breaks down with low values, 
     * unless you adjust the pre-scaler, but the encoder resolution is limited,
     * so you only want to go so fast.
    */
    uint8_t highbits = top / 256;
    uint8_t lowbits = top - highbits;
    TC4H = highbits; OCR4C = lowbits;

    // Enable overflow interrupt
    TIMSK4 = 0x04; 

    /**
     * Uncommenting the following lines will pipe the timer signal to pin 6, 
     * which controls the buzzer. The pin will toggle at the loop rate, which
     * allows you to check that the timing is correct. It will also make a lot
     * of noise, so do so sparingly.
     */
    // TCCR4C = 0x04
    // pinMode(6, OUTPUT);

    // Re-enable interrupts
    sei(); 

    Serial.println("/InitTimer");
}

void Chassis::InititalizeChassis(void)
{
    InitializeMotorControlTimer();
    InitializeMotors();
    
    //Emu
    imu.init();
    imu.setGyroDataOutputRate(LSM6::ODR208);
    imu.setAccDataOutputRate(LSM6::ODR208);
}

/**
 * The main Chassis loop.
 */
bool Chassis::ChassisLoop(Twist& velocity)
{
    bool retVal = false;

    if(loopFlag)
    {
        if(loopFlag > 1) Serial.println("Missed an update in Robot::RobotLoop()!");

#ifdef __LOOP_DEBUG__
        Serial.print(millis());
        Serial.print('\n');
#endif

        // motor updates
        UpdateMotors();

        /* Update the wheel velocity so it gets back to Robot. */
        velocity = CalcOdomFromWheelMotion();

        loopFlag = 0;

        retVal = true;
    }

    return retVal;
}

/**
 * Some motor methods.
 */
void Chassis::InitializeMotors(void)
{
    Romi32U4MotorBase::InitializePWMTimerAndInterrupts();

    leftMotor.InitializeMotor();
    rightMotor.InitializeMotor();
}

void Chassis::SetMotorEfforts(int16_t left, int16_t right) 
{
    leftMotor.SetMotorEffortDirect(left); 
    rightMotor.SetMotorEffortDirect(right);
}

void Chassis::UpdateMotors(void)
{
    leftMotor.ControlMotorSpeed();
    rightMotor.ControlMotorSpeed();
}

/**
 * SetWheelSpeeds converts the linear wheel speeds (axes relative to ground) to motor speeds.
 */
void Chassis::SetWheelSpeeds(float leftSpeedCMperSec, float rightSpeedCMperSec)
{
    /** 
     * TODO: Check the code below. You did this in Lab 1, so not repeated here.
     */
    leftMotor.SetTargetSpeed(leftSpeedCMperSec * LEFT_TICKS_PER_CM * CONTROL_LOOP_PERIOD_MS / 1000.);
    rightMotor.SetTargetSpeed(rightSpeedCMperSec * RIGHT_TICKS_PER_CM * CONTROL_LOOP_PERIOD_MS / 1000.);
}

void Chassis::SetTwist(const Twist& twist)
{
    /**
     * TODO: Complete SetTwist() to call SetWheelSpeeds() from target u and omega
     */
    float linSpeedRight = twist.u + twist.omega * ROBOT_RADIUS;
    float linSpeedLeft = twist.u - twist.omega * ROBOT_RADIUS;
    SetWheelSpeeds(linSpeedRight, linSpeedLeft);
}

Twist Chassis::CalcOdomFromWheelMotion(void)
{
    Twist velocity;
    /**
     * TODO: Calculate velocities from wheel motion
     */

    //                     counts/interval  /       counts/cm          *   intervals/s          =       cm/s
    float linSpeedRight = rightMotor.speed / float(RIGHT_TICKS_PER_CM) * (1000.0 / CONTROL_LOOP_PERIOD_MS);
    float linSpeedLeft = leftMotor.speed / float(LEFT_TICKS_PER_CM) * (1000.0 / CONTROL_LOOP_PERIOD_MS);
 
    velocity.u = (linSpeedRight + linSpeedLeft) / 2;
    velocity.omega = (linSpeedRight - linSpeedLeft) / (ROBOT_RADIUS*2);

#ifdef __NAV_DEBUG__
    TeleplotPrint("u", velocity.u);
    TeleplotPrint("omega", velocity.omega);
#endif

    return velocity;
}

bool Chassis::CheckIMU(float& currentAngleEst)
{
    bool retVal = false;

    if(imu.checkForNewData())
    {
    #ifdef __IMU_DEBUG__
        imu.TeleplotPrint("a.x", imu.a.x);
        imu.TeleplotPrint("a.y", imu.a.y);
        imu.TeleplotPrint("a.z", imu.a.z);
        imu.TeleplotPrint("g.x", imu.g.x);
        imu.TeleplotPrint("g.y", imu.g.y);
        imu.TeleplotPrint("g.z", imu.g.z);
    #endif

        /* from accelerometer */
        acc_x_avg = 0;
        acc_z_avg = 0;

        for(int i = 0; i < 4; i++) {
            acc_x_readings[i] = acc_x_readings[i + 1];
            acc_z_readings[i] = acc_z_readings[i + 1];

            acc_x_avg += acc_x_readings[i];
            acc_z_avg += acc_z_readings[i];
        }
        acc_x_readings[4] = imu.a.x;
        acc_z_readings[4] = imu.a.z;
        acc_x_avg = (acc_x_avg + imu.a.x) / 5;
        acc_z_avg = (acc_z_avg + imu.a.z) / 5;

        acc_angle = degrees(atan2(acc_x_avg, acc_z_avg));

        // if(acc_angle < 0) { //for 0 - 360 deg
        //     acc_angle += 360;
        // } 
        
        /* from gyro */
        gyro_y_avg = 0;

        for(int i = 0; i < 4; i++) {
            gyro_y_readings[i] = gyro_y_readings[i + 1];
            gyro_y_avg += gyro_y_readings[i];
        }
        gyro_y_readings[4] = imu.g.y;
        gyro_y_avg = -(((gyro_y_avg + imu.g.y) / 5) + 765); //gyro bias 0f 790.726

        // if(gyro_y_avg < 0) { //for 0 -360 deg
        //     gyro_y_avg += 360;
        // }

        // Integration of gyro readings compared to previous reading
        gyro_y_angle = gyro_y_prev + ((1.0 / 208.0) * (0.00875 * gyro_y_avg)); // 0.00875dps/LSB, 1/208 update period
        gyro_y_prev = gyro_y_angle;


        /* putting it all together */
        estimatedPitchAngle = (kappa * acc_angle) + ((1.0 - kappa) * gyro_y_angle);

        // Serial.print("estimated pitch angle: "); Serial.print(estimatedPitchAngle);
        // Serial.print(", kappa value: "); Serial.println(kappa);

        currentAngleEst = estimatedPitchAngle;
        retVal = true;
    }

    return retVal;
}