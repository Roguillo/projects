#include "chassis.h"
#include "Romi32U4MotorTemplate.h"

Romi32U4EncodedMotor<LEFT_XOR, LEFT_B, PWM_L, DIR_L, OCR_L> leftMotor("L");
Romi32U4EncodedMotor<RIGHT_XOR, RIGHT_B, PWM_R, DIR_R, OCR_R> rightMotor("R");

/**
 * Because it's declared static, we initialize Chassis::loopFlag here.
 */
uint8_t Chassis::loopFlag = 0;

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
