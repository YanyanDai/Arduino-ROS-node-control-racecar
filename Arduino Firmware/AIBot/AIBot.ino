#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#define USB_USBCON
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nodeHandle;

const int minSteering = 45 ;
const int maxSteering = 135 ;
const int minThrottle = 50 ;
const int maxThrottle = 255 ;

//const int DIRPin[2] = {2, 4}; //L298N
//const int PWMPin = 5; //L298N
const int DIRPin = 4; //yfrobot motor driver PM-R3
const int PWMPin = 5;  //yfrobot motor driver 
const int Forward = 0; //yfrobot motor driver 
const int Backward = 1; //yfrobot motor driver

Servo steeringServo;


// Arduino 'map' funtion for floating point
double fmap (double toMap, double in_min, double in_max, double out_min, double out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void driveCallback ( const geometry_msgs::Twist&  twistMsg )
{
  
  int steeringAngle = fmap(twistMsg.angular.z, -0.5, 0.5, minSteering, maxSteering) ;

  if (steeringAngle < minSteering) { 
    steeringAngle = minSteering;
  }
  if (steeringAngle > maxSteering) {
    steeringAngle = maxSteering ;
  }
  steeringServo.write(steeringAngle) ;
  
  // ESC forward is between 0.5 and 1.0
  int escCommand ;
  escCommand = (int)fmap(abs(twistMsg.linear.x), 0.5, 1.0, minThrottle, maxThrottle) ;

  // Check to make sure throttle command is within bounds
  if (escCommand < minThrottle) { 
    escCommand = minThrottle;
  }
  if (escCommand > maxThrottle) {
    escCommand = maxThrottle ;
  }
  if (twistMsg.linear.x >= 0.5){
  //digitalWrite(DIRPin[0],LOW);
  //digitalWrite(DIRPin[1],HIGH);
    digitalWrite(DIRPin,Forward);
  }else{
  //digitalWrite(DIRPin[0],HIGH);
  //digitalWrite(DIRPin[1],LOW);
    digitalWrite(DIRPin,Backward);
  }
  analogWrite(PWMPin,escCommand);

}

ros::Subscriber<geometry_msgs::Twist> driveSubscriber("/aibot/cmd_vel", &driveCallback) ;

void setup(){
  //for (int i=0; i<2; i++){
  //pinMode(DIRPin[i], OUTPUT);}
  pinMode(DIRPin, OUTPUT);
  pinMode(PWMPin, OUTPUT);
  Serial.begin(115200) ;
  // Intial Node
  nodeHandle.initNode();
  // Subscribe to the steering and throttle messages
  nodeHandle.subscribe(driveSubscriber) ;
  // Attach the servos to actual pins
  //steeringServo.attach(9); // Steering servo is attached to pin 9
  steeringServo.attach(8); // Steering servo is attached to pin 8
  // Steering centered is 90
  steeringServo.write(90) ;
  //digitalWrite(DIRPin[0],LOW);
  //digitalWrite(DIRPin[1],HIGH);
  digitalWrite(DIRPin,Forward);
  analogWrite(PWMPin,0);
  delay(1000) ;
  
}

void loop(){
  nodeHandle.spinOnce();
}
