/*
 * rosserial Servo Control
 *
 * This sketch demonstrates the control of Hs422 servos
 * using ROS and the arduiono
 * 
 */

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle  nh;

Servo servoThumb;
Servo servoIndex;
Servo servoMiddle;
Servo servoRaP;


void servo_Drive(const std_msgs::Int16MultiArray & cmd_msg){
 
  //set servo angle, should be from 0-180
  servoThumb.write(cmd_msg.data[0]);  
  servoIndex.write(cmd_msg.data[1]);  
  servoMiddle.write(cmd_msg.data[2]);  
  servoRaP.write(cmd_msg.data[3]);  
  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}
ros::Subscriber<std_msgs::Int16MultiArray> sub("servo", servo_Drive);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);

//Servo pins attached
  servoThumb.attach(3);
  servoIndex.attach(4);
  servoMiddle.attach(5);
  servoRaP.attach(6);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
