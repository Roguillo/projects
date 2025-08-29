#include <Arduino.h>
#include <Servo.h>
#include <Romi32U4.h>
#include <math.h>

//Romi hardware
Romi32U4Encoders encoders;
Romi32U4Motors motors;

//servo
Servo black_servo;
Servo micro_servo;

//motor pins
#define BDC_ENCODER 0
#define BDC_PWM 11
#define BDC_AIN1 13
#define BDC_AIN2 4
#define BLACK_SERVO_PIN 5
#define MICRO_SERVO_PIN 6

//conversion
const double BDC_pulses_to_deg = 0.139;
const double romi_pulses_to_cm = 0.0179;
const double romi_pulses_to_deg = 0.113;

//>>> five bar
//link lengths
int OC = 90; //mm
int OA = 40; //mm
int AP = 75; //mm
int BC = 80; //mm
int BP = 60; //mm

//gripper pos
int gripper_x_pos = 0;
int gripper_y_pos = 100;

//motor angles
double theta2; //deg
double theta4; //deg
int micro_servo_pos = 1472; //ms (midpoint/90deg)

//BDC
volatile long BDC_pulse_count = 648;
double BDC_target_angle = 90;
double BDC_kp = 18.0;
double BDC_kd = 0.0;
double BDC_ki = 0.0;
double BDC_it = 0.0;
double BDC_prev_error = 0.0;
unsigned long BDC_prev_time = 0;
bool BDC_direction = true; //true for clockwise, false for counterclockwise

bool romi_set_angle = false; //true to set angle, false to set pos
int romi_speed_left = 0;
int romi_speed_right = 0;
int romi_speed_limit = 50;
double romi_kp = 10;
double romi_real_pos_left = 0;
double romi_real_pos_right = 0;
double romi_real_angle_left = 0;
double romi_real_angle_right = 0;
double romi_target_pos = 0;
double romi_target_angle = 0;

//keyboard key
int key;



/**
 * counts BDC encoder pulses
 * will count forwards or backwards depending on BDC_direction
 */
void BDC_pulse_counter() {
  if (BDC_direction) {
    BDC_pulse_count++;

  } else {
    BDC_pulse_count--;
  }
}

//BDC functions
/**
 * dc     duty cycle (%)
 *
 * moves BDC clockwise at the specified duty cycle
 */
void BDC_forwards(int dc) {
  digitalWrite(BDC_AIN1, HIGH);
  digitalWrite(BDC_AIN2, LOW);

  int val = constrain((dc * 255) / 100, 0, 255);
  analogWrite(BDC_PWM, val);

  BDC_direction = true;
}

/**
 * dc     duty cycle (%)
 *
 * moves BDC counterclockwise at the specified duty cycle
 */
void BDC_backwards(int dc) {
  digitalWrite(BDC_AIN1, LOW);
  digitalWrite(BDC_AIN2, HIGH);

  int val = constrain((dc * 255) / 100, 0, 255);
  analogWrite(BDC_PWM, val);

  BDC_direction = false;
}

/**
 * stops BDC, sets dc to 0
 */
void BDC_stop() {
  digitalWrite(BDC_AIN1, LOW);
  digitalWrite(BDC_AIN2, LOW);
  analogWrite(BDC_PWM, 0);
}


/**
 * x     distance to displace gripper by in x-direction
 * y     distance to displace gripper by in y-direction
 *
 * origin at the second motor
 */
void five_bar(int x, int y) {
  gripper_x_pos += x;
  gripper_y_pos += y;

  double OP = sqrt(sq(gripper_x_pos + (OC / 2.0)) + sq(gripper_y_pos));
  double gamma = atan2((double)gripper_y_pos, gripper_x_pos + (OC / 2.0));
  double delta = acos(constrain((sq(OA) + sq(OP) - sq(AP)) / (2.0 * OA * OP), -1.0, 1.0));

  theta2 = degrees(gamma + delta);
  theta2 = constrain(theta2, 33, 150);

  double x_on_SM = gripper_x_pos - (OC / 2.0);
  double y_on_SM = gripper_y_pos;

  Serial.print("gripper x-pos = ");
  Serial.print(x_on_SM);
  Serial.print(", gripper y-pos = ");
  Serial.println(y_on_SM);

  double OC = sqrt(sq(x_on_SM) + sq(y_on_SM));
  double alpha = atan2(y_on_SM, x_on_SM);
  double beta = acos(constrain((sq(BC) + sq(OC) - sq(BP)) / (2.0 * BC * OC), -1.0, 1.0));

  theta4 = degrees(alpha - beta);
  theta4 = constrain(theta4, 67, 172);

  black_servo.write(theta2);
  BDC_target_angle = theta4;
}


 /** 
  * >>> controls 
  * 
  * > Navigation 
  * w :   -> set position target to 2cm forwards 
  * 3 :   -> set position target to 5cm forwards 
  * a :   -> set angle target to 10deg counterclockwise 
  * d :   -> set angle target to 10deg clockwise 
  * s :   -> set position target to 2cm backwards 
  * x :   -> set position target to 5cm backwards 
  * 

  * > Gripper 
  * i :   -> move gripper forwards 
  * k :   -> move gripper backwards 
  * j :   -> move gripper upwards 
  * l :   -> move gripper downwards 
  * u :   -> open gripper 
  * o :   -> close gripper 
  * 

  * > Other 
  * e :   -> detach servos, stop BDC 
  * q :   -> reattach servos 
  * r :   -> reset romi motor encoders and associated values 
  */ 
void keyboard() {
  while (Serial.available()) {
  key = Serial.read();

  switch (key) {
    case 'w':
      romi_target_pos += 2;
      romi_set_angle = false;
      Serial.println("2cm forwards");
      break;

    case '3':
      romi_target_pos += 5;
      romi_set_angle = false;
      Serial.println("5cm forwards");
      break;

    case 's':
      romi_target_pos -= 2;
      romi_set_angle = false;
      Serial.println("2cm backwards");
      break;

    case 'x':
      romi_target_pos -= 5;
      romi_set_angle = false;
      Serial.println("5cm backwards");
      break;

    case 'a':
      romi_target_angle -= 5;
      romi_set_angle = true;
      Serial.println("5 deg counterclockwise");
      break;

    case 'd':
      romi_target_angle += 5;
      romi_set_angle = true;
      Serial.println("5 deg clockwise");
      break;

    case 'k': 
      five_bar(10, 0);
      Serial.println("move gripper forwards");
      break;

    case 'i':
      five_bar(-10, 0);
      Serial.println("move gripper backwards");
      break;

    case 'j':
      five_bar(0, 10);
      Serial.println("move gripper upwards");
      break;

    case 'l':
      five_bar(0, -10);
      Serial.println("move gripper downwards");
      break;

    case 'u':
      micro_servo.writeMicroseconds(1150);
      Serial.println("open gripper");
      break;

    case 'o':
      micro_servo.writeMicroseconds(1650);
      Serial.println("close gripper");
      break;

    case 'e':
      black_servo.detach();
      micro_servo.detach();
      BDC_stop();
      Serial.println("detach servos, stop BDC");
      break;

    case 'q':
      black_servo.attach(BLACK_SERVO_PIN);
      micro_servo.attach(MICRO_SERVO_PIN);
      Serial.println("reattach servos");
      break;

    case 'r':
      romi_real_pos_left = encoders.getCountsAndResetLeft() * romi_pulses_to_cm;
      romi_real_pos_right = encoders.getCountsAndResetRight() * romi_pulses_to_cm;
      romi_real_angle_left = encoders.getCountsAndResetLeft() * romi_pulses_to_deg;
      romi_real_angle_right = encoders.getCountsAndResetRight() * romi_pulses_to_deg;
      romi_target_pos = 0;
      romi_target_angle = 0;
      romi_speed_left = 0;
      romi_speed_right = 0;
      Serial.println("reset romi motor encoders");
      break;
    }
  }
}


//BDC PID control
void BDC_PID_controller() {
  unsigned long curr_time = millis();
  double dt = (curr_time - BDC_prev_time) / 1000.0;

  if (dt >= 0.01) {
    double BDC_curr_angle = BDC_pulse_count * BDC_pulses_to_deg;
    double error = BDC_target_angle - BDC_curr_angle;
    BDC_it += error * dt;
    double BDC_derv = (error - BDC_prev_error) / dt;
    double effort = BDC_kp * error + BDC_ki * BDC_it + BDC_kd * BDC_derv;
    int val = constrain(abs((int)effort), 0, 100);

    if (effort > 0) {
      BDC_direction = true;
      BDC_forwards(val);

    } else if (effort < 0) {
      BDC_direction = false;
      BDC_backwards(val);

    } else {
      BDC_stop();
    }

    BDC_prev_error = error;
    BDC_prev_time = curr_time;

    Serial.print("BDC Target: "); Serial.print(BDC_target_angle);
    Serial.print(" deg, Real: "); Serial.print(BDC_curr_angle);
    Serial.print(" deg, Error: "); Serial.print(error);
    Serial.print(" deg, PWM: "); Serial.println(val);
  }
}

//Romi navigation
void nav() {
  romi_real_pos_left = encoders.getCountsLeft() * romi_pulses_to_cm;
  romi_real_pos_right = encoders.getCountsRight() * romi_pulses_to_cm;
  romi_real_angle_left = encoders.getCountsLeft() * romi_pulses_to_deg;
  romi_real_angle_right = encoders.getCountsRight() * romi_pulses_to_deg;

  if (!romi_set_angle) {
    romi_speed_left = (romi_target_pos - romi_real_pos_left) * romi_kp;
    romi_speed_right = (romi_target_pos - romi_real_pos_right) * romi_kp;

  } else {
    romi_speed_left = (romi_target_angle - romi_real_angle_left) * romi_kp;
    romi_speed_right = -(romi_target_angle - romi_real_angle_left) * romi_kp;
  }

  int16_t speed_cap = (300 * romi_speed_limit) / 100;

  if (romi_speed_left < speed_cap && romi_speed_left > -speed_cap) {
    motors.setLeftSpeed(romi_speed_left);

  } else if (romi_speed_left > speed_cap) {
    motors.setLeftSpeed(speed_cap);

  } else if (romi_speed_left < -speed_cap) {
    motors.setLeftSpeed(-speed_cap);
  }

  if (romi_speed_right < speed_cap && romi_speed_right > -speed_cap) {
    motors.setRightSpeed(romi_speed_right);

  } else if (romi_speed_right > speed_cap) {
   motors.setRightSpeed(speed_cap);

  } else if (romi_speed_right < -speed_cap) {
    motors.setRightSpeed(-speed_cap);
  }
}



void setup() {
  black_servo.attach(BLACK_SERVO_PIN);
  micro_servo.attach(MICRO_SERVO_PIN);

  black_servo.writeMicroseconds(1472);
  micro_servo.writeMicroseconds(micro_servo_pos);

  pinMode(BDC_ENCODER, INPUT);
  pinMode(BDC_PWM, OUTPUT);
  pinMode(BDC_AIN1, OUTPUT);
  pinMode(BDC_AIN2, OUTPUT);

  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(BDC_ENCODER), BDC_pulse_counter, RISING);

  BDC_prev_time = millis();
}

void loop() {
  keyboard();
  nav();
  BDC_PID_controller();
}