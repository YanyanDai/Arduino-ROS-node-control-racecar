#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;
geometry_msgs::Twist cmd_vel; //모터 명령 수신을 위한 변수(수신)
std_msgs::Int32 encoder_data1; //모터 엔코터1 값 전달을 위한 변수 (송신)
std_msgs::Int32 encoder_data2; //모터 엔코터2 값 전달을 위한 변수 (송신)

int velocity=0;
int steer_angle = 0;

void cmd_vel_callback(const geometry_msgs::Twist&  msg){
  velocity = (int)msg.linear.x; //속도 제어
  steer_angle = (int)msg.angular.z; //steering motor 각도 제어

  if (velocity >= 255) velocity = 255; //PWM 최고값 제한
  if (velocity <= -255) velocity = -255; //PWM 최저값 제한
  }

  ros::Subscriber<geometry_msgs::Twist> cmd_sub("teleop_cmd_vel", cmd_vel_callback);
  ros::Publisher cmd_pub("cmd_vel2", &cmd_vel);
  ros::Publisher encoder_pub1("encoder1", &encoder_data1);
  ros::Publisher encoder_pub2("encoder2", &encoder_data2);


void setup() {
  // put your setup code here, to run once:
  // Front Motor Drive Pin Setup
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR1_ENB, OUTPUT);

  // Rear Motor Drive Pin Setup
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR2_ENB, OUTPUT);

  nh.initNode();
  nh.subscribe(cmd_sub); //subscribing cmd_vel
  nh.advertise(cmd_pub); //publishing cmd_vel2
}

void loop() {
  // put your main code here, to run repeatedly:

  int f_speed = 0, r_speed = 0;
  f_speed = r_speed = velocity;
  motor_control(f_speed, r_speed);
  // cmd_vel 수신을 다시 pub하여 확인 하기 위한 루틴
  cmd_vel.linear.x = velocity;
  cmd_vel.angular.z = steer_angle;
  cmd_pub.publish(&cmd_vel);
  nh.spinOnce();
}
