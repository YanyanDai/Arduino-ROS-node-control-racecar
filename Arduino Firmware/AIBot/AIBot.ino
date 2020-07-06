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

const int minSteering = 0 ;
const int maxSteering = 90 ;
const int minThrottle = 0 ;
//const int minThrottle = 110 ;
const int maxThrottle = 150 ;

const int DIRPin = 4;
const int PWMPin = 5;
const int Forward = 1;
const int Backward = 0;

Servo steeringServo;
//Servo electronicSpeedController ;  // The ESC on the TRAXXAS works like a Servo

std_msgs::Int32 str_msg;
ros::Publisher chatter("chatter", &str_msg); 

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
  escCommand = (int)fmap(abs(twistMsg.linear.x), 0.5, 1.0, 110.0, maxThrottle) ;

  // Check to make sure throttle command is within bounds
  if (escCommand < minThrottle) { 
    escCommand = minThrottle;
  }
  if (escCommand > maxThrottle) {
    escCommand = maxThrottle ;
  }
  if (twistMsg.linear.x >= 0.5){
  digitalWrite(DIRPin,Forward);
  }else{
  digitalWrite(DIRPin,Backward);
  }
  analogWrite(PWMPin,escCommand);

}

ros::Subscriber<geometry_msgs::Twist> driveSubscriber("/aibot/cmd_vel", &driveCallback) ;

void setup(){
  pinMode(13, OUTPUT);
  pinMode(DIRPin, OUTPUT);
  pinMode(PWMPin, OUTPUT);
  Serial.begin(115200) ;
  nodeHandle.initNode();
  // This can be useful for debugging purposes
  nodeHandle.advertise(chatter);
  // Subscribe to the steering and throttle messages
  nodeHandle.subscribe(driveSubscriber) ;
  // Attach the servos to actual pins
  steeringServo.attach(9); // Steering servo is attached to pin 9
  // Steering centered is 45
  steeringServo.write(45) ;
  digitalWrite(DIRPin,Forward);
  analogWrite(PWMPin,0);
  delay(1000) ;
  
}

void loop(){
  nodeHandle.spinOnce();
  delay(1);
}
