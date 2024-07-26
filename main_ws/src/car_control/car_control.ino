#include <MsTimer2.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

int velocity = 0;
float steer_angle = 0.0;

void linear_xz(const geometry_msgs::Twist& msg) {
  velocity = (int)msg.linear.x;       // 속도제어
  steer_angle = (int)msg.angular.z;   // steering motor 각도 제어

  if(velocity >= 255) velocity  = 255;  // pwm 최고값 제한
  if(velocity <=-255) velocity = -255;  // pwm 최저값 제한
}

ros::Subscriber<geometry_msgs::Twist> control_value_sub("/control_value", linear_xz);
// Front Motor Drive
#define MOTOR1_PWM 2
#define MOTOR1_ENA 3
#define MOTOR1_ENB 4

#define MOTOR2_PWM 5
#define MOTOR2_ENA 6
#define MOTOR2_ENB 7
int f_speed = 0, r_speed = 0;

void front_motor_control(int motor1_pwm)
{
  if (motor1_pwm > 0) // forward
  {
    digitalWrite(MOTOR1_ENA, HIGH);
    digitalWrite(MOTOR1_ENB, LOW);
    analogWrite(MOTOR1_PWM, motor1_pwm);
  }
  else if (motor1_pwm < 0) // backward
  {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, HIGH);
    analogWrite(MOTOR1_PWM, -motor1_pwm);
  }
  else
  {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, LOW);
    digitalWrite(MOTOR1_PWM, 0);
  }
}

void rear_motor_control(int motor2_pwm)
{
  if (motor2_pwm > 0) // forward
  {
    digitalWrite(MOTOR2_ENA, HIGH);
    digitalWrite(MOTOR2_ENB, LOW);
    analogWrite(MOTOR2_PWM, motor2_pwm);
  }
  else if (motor2_pwm < 0) // backward
  {
    digitalWrite(MOTOR2_ENA, LOW);
    digitalWrite(MOTOR2_ENB, HIGH);
    analogWrite(MOTOR2_PWM, -motor2_pwm);
  }
  else
  {
    digitalWrite(MOTOR2_ENA, LOW);
    digitalWrite(MOTOR2_ENB, LOW);
    digitalWrite(MOTOR2_PWM, 0);
  }
}

void motor_control(int front_speed, int rear_speed)
{
  front_motor_control(front_speed);
  rear_motor_control(rear_speed);  
}

///////////////////////////////////////  Steering PID 제어 /////////////////////////////////////////////
#define Steering_Sensor A15  // Analog input pin that the potentiometer is attached to
#define NEURAL_ANGLE 0
#define LEFT_STEER_ANGLE -25
#define RIGHT_STEER_ANGLE 25
#define MOTOR3_PWM 8
#define MOTOR3_ENA 9
#define MOTOR3_ENB 10
#define AD_MIN (40 + 10)
#define AD_MAX (940 - 10)

float Kp = 7.5;
float Ki = 0.0;
float Kd = 4.5; // PID 상수 설정, 실험에 따라 정해야 함 중요!
double Setpoint, Input, Output; // PID 제어 변수
double error, error_old;
double error_s, error_d;
int pwm_output;

int sensorValue = 0;        // value read from the pot
int Steer_Angle_Measure = 0;        // value output to the PWM (analog out)
int Steering_Angle = NEURAL_ANGLE;

void steer_motor_control(int motor_pwm)
{
  if ((sensorValue >= AD_MAX) || (sensorValue <= AD_MIN))
  {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, 0);
    return;
  }

  if (motor_pwm > 0) // forward
  {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, HIGH);
    analogWrite(MOTOR3_PWM, motor_pwm);
  }
  else if (motor_pwm < 0) // backward
  {
    digitalWrite(MOTOR3_ENA, HIGH);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, -motor_pwm);
  }
  else // stop
  {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, 0);
  }
}

void PID_Control()
{
  error = Steering_Angle - Steer_Angle_Measure;
  error_s += error;
  error_d = error - error_old;
  error_s = (error_s >= 100) ? 100 : error_s;
  error_s = (error_s <= -100) ? -100 : error_s;

  pwm_output = Kp * error + Kd * error_d + Ki * error_s;
  pwm_output = (pwm_output >= 255) ? 255 : pwm_output;
  pwm_output = (pwm_output <= -255) ? -255 : pwm_output;

  if (fabs(error) <= 0.2)  // 특정 값 이하면 제어를 멈추어서 조정이 안되도록
  {
    steer_motor_control(0);
    error_s = 0;
  }
  else
  {
    steer_motor_control(pwm_output);
  }
  error_old = error;  
}

void steering_control()
{
  if (Steering_Angle <= LEFT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle = LEFT_STEER_ANGLE + NEURAL_ANGLE;
  if (Steering_Angle >= RIGHT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle = RIGHT_STEER_ANGLE + NEURAL_ANGLE;
  PID_Control(); 
}

void control_callback()
{
  static boolean output = HIGH;

  digitalWrite(13, output);
  output = !output;

  motor_control(f_speed, r_speed);

  // read the analog in value:
  sensorValue = analogRead(Steering_Sensor);
  Steer_Angle_Measure = map(sensorValue, 50, 950, LEFT_STEER_ANGLE, RIGHT_STEER_ANGLE);
  Steering_Angle = NEURAL_ANGLE + steer_angle;
  steering_control();  
}

void setup() {
  // put your setup code here, to run once:
//  nh.getHardware()->setBaud(9600);
  pinMode(13, OUTPUT);
  // Front Motor Drive Pin Setup
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR1_ENB, OUTPUT);

  // Rear Motor Drive Pin Setup
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR2_ENB, OUTPUT); 

  // Steer
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR3_ENB, OUTPUT);  // L298 motor control PWM

  error = error_s = error_d = error_old = 0.0;
  pwm_output = 0;

  nh.initNode();
  nh.subscribe(control_value_sub);

  MsTimer2::set(100, control_callback); // 500ms period
  MsTimer2::start();
}

void loop() {
  // put your main code here, to run repeatedly
  f_speed = r_speed = velocity;

  delay(20);
  nh.spinOnce();
}
