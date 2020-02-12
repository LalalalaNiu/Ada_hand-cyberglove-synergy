/*
 * rosserial Servo Control & the FSR sensor read out;
 *
 * This sketch demonstrates the control of Hs422 servos, The analog data from the FSR senor.
 * using ROS and the arduiono serial 
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

// FSR is connected to analog 0~5 0 for plam 1-5 from Thumb to pinky
#define fsrPalmPin A0
#define fsrThumbPin A1 
#define fsrIndexPin A2
#define fsrMiddlePin A3
#define fsrRingPin A4
#define fsrPinkyPin A5

// Define variables:
int fsr0,fsr1,fsr2,fsr3,fsr4,fsr5; // The current reading from the FSR
Servo servoThumb;
Servo servoIndex;
Servo servoMiddle;
Servo servoRaP;

ros::NodeHandle  nh;
//Publish node;
std_msgs::Int16MultiArray Fsr_msg;
ros::Publisher pub("Fsr/sensor_msgs", &Fsr_msg);

void servo_Drive( const std_msgs::Int16MultiArray & cmd_msg){
 
  //set servo angle, should be from 0-180
  servoThumb.write(cmd_msg.data[0]);  
  servoIndex.write(cmd_msg.data[1]);  
  servoMiddle.write(cmd_msg.data[2]);  
  servoRaP.write(cmd_msg.data[3]);   
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("servo", servo_Drive);

void setup() {
//    
 
  nh.initNode();
  
  Fsr_msg.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension)*2);
  Fsr_msg.layout.dim[0].label = "height";
  Fsr_msg.layout.dim[0].size = 6;
  Fsr_msg.layout.dim[0].stride = 1;
  Fsr_msg.layout.data_offset = 0;
  Fsr_msg.data = (int *)malloc(sizeof(int)*8);
  Fsr_msg.data_length = 6;

  //Servo pins attached on digital port:
  servoThumb.attach(3);
  servoIndex.attach(4);
  servoMiddle.attach(5);
  servoRaP.attach(6);
  
  nh.advertise(pub);
  nh.subscribe(sub);
}

void loop() {
  // Read the FSR pin and store the output as fsrreading:
  fsr0 = analogRead(fsrPalmPin);
  fsr1 = analogRead(fsrThumbPin);
  fsr2 = analogRead(fsrIndexPin);
  fsr3 = analogRead(fsrMiddlePin);
  fsr4 = analogRead(fsrRingPin);
  fsr5 = analogRead(fsrPinkyPin);
  // Print the fsrreading in the serial monitor:
  //Serial.println(fsr0);
  //remap~~
  
  Fsr_msg.data[0] = fsr0;
  Fsr_msg.data[1] = fsr1;
  Fsr_msg.data[2] = fsr2;
  Fsr_msg.data[3] = fsr3;
  Fsr_msg.data[4] = fsr4;
  Fsr_msg.data[5] = fsr5;
  //pubnilsh the FSR data 
  
  pub.publish(&Fsr_msg);
  nh.spinOnce();
  delay(10);
}
   