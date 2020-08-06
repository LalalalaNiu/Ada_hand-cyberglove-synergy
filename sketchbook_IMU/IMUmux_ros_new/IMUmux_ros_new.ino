

/*
 * IMU mux implementation
 * To be used on the openbionics robot hand sensors
 */
#include <ros.h>
#include <ros/time.h>
#include <th_messages/raw_imu.h>
#include <Wire.h>
#include "LSM1MUX.h"

#define NUM_SENSORS 1
#define LSM9DS1_XG  0x6A // Would be 0x6B if SDO_XG is LOW
#define LSM9DS1_M   0x0E // Would be 0x0F if SDO_M is LOW

ros::NodeHandle nh;

uint16_t status[NUM_SENSORS];
LSM9DS1 dof(MODE_I2C, LSM9DS1_XG, LSM9DS1_M);

th_messages::raw_imu rimu;
ros::Publisher rir_reads("raw_imus", &rimu);

long seq=0;
float pressure = 0, tempe = 0;

void setup() {
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(12, LOW);
  digitalWrite(11, LOW);
  digitalWrite(10, LOW);
  digitalWrite(9, LOW);
  pinMode(13, OUTPUT);

  digitalWrite(13, HIGH);
  delay(750);
  digitalWrite(13, LOW);
  delay(750);
  
  for(short i = 0; i < NUM_SENSORS; i++){
    status[i] = dof.beginID(i);
    dof.setAccelScale(dof.A_SCALE_2G);
    dof.setAccelODR(dof.A_ODR_238); //new sensor ODR value for ACCEL
    dof.setAccelABW(dof.A_ABW_50);
    dof.setGyroScale(dof.G_SCALE_245DPS);
    dof.setGyroAccelODR(dof.G_ODR_2380_BW_78); //new sensor ODR and band-width values for GYRO
    dof.setMagScale(dof.M_SCALE_4GS); //the new sensor doesnt have 2GS
    dof.setMagODR(dof.M_ODR_10); //new sensor ODR value for MAG
    dof.calLSM9DS1(i);
  } 

  // put your setup code here, to run once:
  nh.initNode();

  rimu.header.frame_id = "imus_frames";
  nh.advertise(rir_reads);

  delay(5);  
  digitalWrite(13, HIGH);
}

void loop() {

  for(short sensorID = 0; sensorID < NUM_SENSORS; sensorID++){
    dof.readMux(sensorID);

    rimu.sensor_id=sensorID;

    //change this to check the bias!!!
    dof.readTemp(sensorID);
    rimu.tempe = 21.0 + (float) dof.temperature[sensorID]/8.; // slope is 8 LSB per degree C, just guessing at the intercept

    dof.readGyro(sensorID);
    rimu.gx = (float)(dof.calcGyro(dof.gx[sensorID]) - dof.gbias[sensorID][0]);   // Convert to degrees per seconds, remove gyro biases
    rimu.gy = (float)(dof.calcGyro(dof.gy[sensorID]) - dof.gbias[sensorID][1]);
    rimu.gz = (float)(dof.calcGyro(dof.gz[sensorID]) - dof.gbias[sensorID][2]);
    //rimu.gx = dof.gbias[sensorID][0];   // Convert to degrees per seconds, remove gyro biases
    //rimu.gy = dof.gbias[sensorID][1];
    //rimu.gz = dof.gbias[sensorID][2];
  
    dof.readAccel(sensorID);         // Read raw accelerometer data
    rimu.ax = (float)(dof.calcAccel(dof.ax[sensorID]));// - dof.abias[sensorID][0]);   // Convert to g's, remove accelerometer biases
    rimu.ay = (float)(dof.calcAccel(dof.ay[sensorID]));// - dof.abias[sensorID][1]);
    rimu.az = (float)(dof.calcAccel(dof.az[sensorID]));// - dof.abias[sensorID][2]);
    //rimu.ax = dof.abias[sensorID][0];   // Convert to g's, remove accelerometer biases
    //rimu.ay = dof.abias[sensorID][1];
    //rimu.az = dof.abias[sensorID][2];
  
    dof.readMag(sensorID);           // Read raw magnetometer data
    rimu.mx = dof.calcMag(dof.mx[sensorID]);     // Convert to Gauss and correct for calibration
    rimu.my = dof.calcMag(dof.my[sensorID]);
    rimu.mz = dof.calcMag(dof.mz[sensorID]);
  
    rimu.header.seq=seq;
    rimu.header.stamp = nh.now();
    rir_reads.publish( &rimu );
    nh.spinOnce();
  }
  
  seq++;
  //Serial.println("]");
}
