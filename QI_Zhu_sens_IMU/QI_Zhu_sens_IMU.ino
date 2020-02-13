//Written by Ahmet Burkay KIRNIK
//TR_CapaFenLisesi
//Measure Angle with a MPU-6050(GY-521)
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int minVal=265;
int maxVal=402;

double x;
double y;
double z;
double i2=0;
double j=0;
double k=0;
int i;
//pubish node for IMU

ros::NodeHandle  nh;
std_msgs::Int32MultiArray IMUarray;
ros::Publisher pub ("IMU/angle_data", &IMUarray);



void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);
  pinMode(5, OUTPUT); // Enable pin
  pinMode(13, OUTPUT); // Enable pin
  digitalWrite(13, HIGH);
  i=0;
}
void loop(){
 //Serial.print(i); // Changing the enable - AD0 pin of the sensor
 if(i % 2 == 0){
 digitalWrite(5, LOW);
 delay(10);
 digitalWrite(13, HIGH);
 delay(10);
 i=1;
 } else {
 digitalWrite(5, HIGH);
 delay(10);
 digitalWrite(13, LOW);
 delay(10);
 i=0;
 }
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
    int xAng = map(AcX,minVal,maxVal,-90,90);
    int yAng = map(AcY,minVal,maxVal,-90,90);
    int zAng = map(AcZ,minVal,maxVal,-90,90);

       x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
       y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
       z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

//     Serial.print("AngleX= ");
//     Serial.println(x);
//
//     Serial.print("AngleY= ");
//     Serial.println(y);
//
//     Serial.print("AngleZ= ");
//     Serial.println(z);
//     Serial.println("-----------------------------------------");
//     delay(400);

if (i==0){Serial.println(x-i2);}

if (i==1){Serial.println(i2-x);}

//  if (i==0){
//     Serial.print("difference X= ");
//     Serial.println(x-i2);
//
//     Serial.print("difference Y= ");
//     Serial.println(y-j);
//
//     Serial.print("difference Z= ");
//     Serial.println(z-k);
//     Serial.println("-----------------------------------------");
//     //array.data[0]=x-i2;
//     //array.data[1]=y-j;
//     //array.data[2]=z-k;
//     //delay(400);
//     }
//
//  if (i==1){
//     Serial.print("difference X= ");
//     Serial.println(i2-x);
//
//     Serial.print("difference Y= ");
//     Serial.println(j-y);
//
//     Serial.print("difference Z= ");
//     Serial.println(k-z);
//     Serial.println("-----------------------------------------");
     //array.data[0]=i2-x;
     //array.data[1]=j-y;
     //array.data[2]=k-z;
     //delay(400);}
  
    i2=x; // saving the value for the next cicle difference
    j=y;
    k=z;
}
