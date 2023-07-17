/*
 * Brief description
 * 
 */

#include <ros.h>
#include <ESP32Servo.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>

ros::NodeHandle nh;
Servo myservo;

#define SERVO_PIN 12

double posServo1 = 2500; //0
double posServo2 = -2500; //90

//double posServo2 = 1160; //90
double toggle = true; 

double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup()
{
  pinMode(SERVO_PIN, OUTPUT);
  nh.initNode();

  myservo.attach(SERVO_PIN); // PWM pin
  delay(1000);
  
}

void loop()
{
  if(toggle){
    double angle = mapf(0.0, 0.0,1.570796, posServo1, posServo2);
    myservo.writeMicroseconds(angle);
    toggle = false;
  }
  else{
    double angle = mapf(0.785398, 0.0,1.570796, posServo1, posServo2); 
    myservo.writeMicroseconds(angle);
    toggle = true;
  }
  
  
  nh.spinOnce();
  delay(1000);
}
