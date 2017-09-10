#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <Wire.h>
#include "Kalman.h"
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
//#include "robot_specs.h"

//Motor Shield headers
#include <Wire.h>
//#include <Adafruit_MotorShield.h>
//#include "utility/Adafruit_PWMServoDriver.h"

#define encodPinA1      3     // encoder A pin
#define encodPinB1      8     // encoder B pin
#define encodPinA2      2
#define encodPinB2      7
#define LOOPTIME        100   // PID loop time(ms)
#define SMOOTH      10
#define encoder_pulse   500	//13
#define gear_ratio      65 //120
#define wheel_diameter  0.12   //m
#define wheel_width     0.005   //m
#define track_width     0.327   //m
#define MAX_RPM         58
#define pi              3.1415926
#define two_pi          6.2831853
#define rpmtoomg	0.104719755
#define PI 3.1415926535897932
#define RESTRICT_PITCH

#define sign(x) (x > 0) - (x < 0)

// Create the motor shield object with the default I2C address
//Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Select which 'port' M1, M2, M3 or M4. 
//Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
//Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

unsigned long lastMilli = 0;       // loop timing 
unsigned long lastMilliPub = 0;
double rpm_req1 = 0;
double rpm_req2 = 0;
double rpm_act1 = 0;
double rpm_act2 = 0;
double rpm_req1_smoothed = 0;
double rpm_req2_smoothed = 0;
//int direction1 = FORWARD;
//int direction2 = FORWARD;
//int prev_direction1 = RELEASE;
//int prev_direction2 = RELEASE;
int PWM_val1 = 0;
int PWM_val2 = 0;
volatile long count1 = 0;          // rev counter
volatile long count2 = 0;
long countAnt1 = 0;
long countAnt2 = 0;
float Kp =   0.5;
float Kd =   0;
float Ki =   0;
double gyroZ;
double gyroZrad;
//double gyroZrate;
uint8_t i2cData[14];

ros::NodeHandle nh;

void handle_cmd( const geometry_msgs::Twist& cmd_msg) {
  double x = cmd_msg.linear.x;
  double z = cmd_msg.angular.z;
  if (z == 0) {     // go straight
    // convert m/s to rpm
    rpm_req1 = x*60/(pi*wheel_diameter);
    rpm_req2 = rpm_req1;
  }
  else if (x == 0) {
    // convert rad/s to rpm
    rpm_req2 = z*track_width*60/(wheel_diameter*pi*2);
    rpm_req1 = -rpm_req2;
  }
  else {
    rpm_req1 = x*60/(pi*wheel_diameter)-z*track_width*60/(wheel_diameter*pi*2);
    rpm_req2 = x*60/(pi*wheel_diameter)+z*track_width*60/(wheel_diameter*pi*2);
  }
  
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", handle_cmd);
geometry_msgs::Vector3Stamped rpm_msg;
geometry_msgs::Vector3Stamped gyro_msg;
ros::Publisher rpm_pub("rpm", &rpm_msg);
ros::Publisher gyro_pub("gyro", &rpm_msg);
ros::Time current_time;
ros::Time last_time;

void setup() {
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
 //AFMS.begin();  // create with the default frequency 1.6KHz
 count1 = 0;
 count2 = 0;
 countAnt1 = 0;
 countAnt2 = 0;
 rpm_req1 = 0;
 rpm_req2 = 0;
 rpm_act1 = 0;
 rpm_act2 = 0;
 PWM_val1 = 0;
 PWM_val2 = 0;
 nh.initNode();
 nh.getHardware()->setBaud(57600);
 nh.subscribe(sub);
 nh.advertise(rpm_pub);
 nh.advertise(gyro_pub);
  
 pinMode(encodPinA1, INPUT); 
 pinMode(encodPinB1, INPUT); 
 digitalWrite(encodPinA1, HIGH);                // turn on pullup resistor
 digitalWrite(encodPinB1, HIGH);
 attachInterrupt(1, encoder1, RISING);

 pinMode(encodPinA2, INPUT); 
 pinMode(encodPinB2, INPUT); 
 digitalWrite(encodPinA2, HIGH);                // turn on pullup resistor
 digitalWrite(encodPinB2, HIGH);
 attachInterrupt(0, encoder2, RISING);
 //motor1->setSpeed(0);
// motor2->setSpeed(0);
// motor1->run(FORWARD);
 //motor1->run(RELEASE);
// motor2->run(FORWARD);
// motor2->run(RELEASE);
pinMode(5, OUTPUT);
pinMode(6, OUTPUT);
pinMode(10, OUTPUT);
pinMode(11, OUTPUT); 

}

void loop() {
  nh.spinOnce();
  unsigned long time = millis();
 if(time-lastMilli>= LOOPTIME)   {      // enter tmed loop
    getMotorData(time-lastMilli);

    
    if(rpm_req1>14) { rpm_req1=14; }
    if(rpm_req1<-5) { rpm_req1=-5; }
    if(rpm_req2>14) { rpm_req2=14; }
    if(rpm_req2<-5) { rpm_req2=-5; }

    PWM_val1 = updatePid(1, PWM_val1, rpm_req1, rpm_act1);
    PWM_val2 = updatePid(2, PWM_val2, rpm_req2, rpm_act2);

    if(rpm_req1==0 && rpm_act1==0) { PWM_val1=0; }
    if(rpm_req2==0 && rpm_act2==0) { PWM_val2=0; }
    
  
    runmotor1(getdir(PWM_val1)*-1,PWM_val1);
    runmotor2(getdir(PWM_val2),PWM_val2);

///////////GYRO
  while (i2cRead(0x3B, i2cData, 14));
  gyroZ = (i2cData[12] << 8) | i2cData[13];
  //gyroZrate = gyroZ / 131.0; // Convert to deg/s
  gyroZrad = gyroZ*2*PI/(131.0*360);  //Convert to rad/s

////////////////
  
 
  //runmotor1(getdir(rpm_req1)*-1,rpm_req1*rpmtoomg*37.73);
  //runmotor2(getdir(rpm_req2),rpm_req2*rpmtoomg*37.73); 

    //if(PWM_val1 > 0) direction1 = FORWARD;
    //else if(PWM_val1 < 0) direction1 = BACKWARD;
    //if (rpm_req1 == 0) direction1 = RELEASE;
   // if(PWM_val2 > 0) direction2 = FORWARD;
    //else if(PWM_val2 < 0) direction2 = BACKWARD;
   // if (rpm_req2 == 0) direction2 = RELEASE;
    //motor1->run(direction1);
    //motor2->run(direction2);

   // motor1->setSpeed(abs(PWM_val1));
   // motor2->setSpeed(abs(PWM_val2));
    
    publishRPM(time-lastMilli);
    publishGYRO(gyroZrad);
    lastMilli = time;
  }
  if(time-lastMilliPub >= LOOPTIME) {
    publishRPM(time-lastMilliPub);
    publishGYRO(gyroZrad);
    lastMilliPub = time;
  }
  //delay(10);
}

void getMotorData(unsigned long time)  {
 rpm_act2 = double((count1-countAnt1)*60*1000)/double(time*encoder_pulse*gear_ratio)*-1;
 rpm_act1 = double((count2-countAnt2)*60*1000*1.003)/double(time*encoder_pulse*gear_ratio)*-1;
 countAnt1 = count1;
 countAnt2 = count2;
}

//#if 0
int updatePid(int id, int command, double targetValue, double currentValue) {
  double pidTerm = 0;                            // PID correction
  double error = 0;
  double new_pwm = 0;
  double new_cmd = 0;
  static double last_error1 = 0;
  static double last_error2 = 0;
  static double int_error1 = 0;
  static double int_error2 = 0;
  
  error = targetValue-currentValue;
  if (id == 1) {
    int_error1 += error;
    pidTerm = Kp*error + Kd*(error-last_error1) + Ki*int_error1;
    last_error1 = error;
  }
  else {
    int_error2 += error;
    pidTerm = Kp*error + Kd*(error-last_error2) + Ki*int_error2;
    last_error2 = error;
  }
  new_pwm = constrain(double(command)*MAX_RPM/100 + pidTerm, -MAX_RPM, MAX_RPM);
  new_cmd = 100*new_pwm/MAX_RPM;
  return int(new_cmd);
}

//#endif




void publishRPM(unsigned long time) {
  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = rpm_act1;
  rpm_msg.vector.y = rpm_act2;
  rpm_msg.vector.z = double(time)/1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce();
}

void publishGYRO(float time) {
  gyro_msg.header.stamp = nh.now();
  gyro_msg.vector.x = 0;
  gyro_msg.vector.y = 0;
  gyro_msg.vector.z = time;
  gyro_pub.publish(&gyro_msg);
  nh.spinOnce();
}

void encoder1() {
  if (digitalRead(encodPinA1) == digitalRead(encodPinB1)) count1++;
  else count1--;
}
void encoder2() {
  if (digitalRead(encodPinA2) == digitalRead(encodPinB2)) count2--;
  else count2++;
}

void runmotor1(int di,int val)
{ val=abs(val);
  if(di>0)
  { analogWrite(5, 0);
    analogWrite(6, val);  
  }
  else if(di<0)
  { analogWrite(5, val);
    analogWrite(6, 0);  
  }
  else
  { analogWrite(5, 0);
    analogWrite(6, 0);  
  }

}

void runmotor2(int di,int val)
{ val=abs(val);
  if(di>0)
  { analogWrite(10, 0);
    analogWrite(11, val);  
  }
  else if(di<0)
  { analogWrite(10, val);
    analogWrite(11, 0);  
  }
  else
  { analogWrite(10, 0);
    analogWrite(11, 0);  
  }

}

int getdir(float d)
{ int di;
  if(d>0)
  { di=1;	}
  else if(d<0)
  { di=-1;	}
  else
  { di=0;	}

  return int(di);
}
