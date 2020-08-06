

/*
 * Barometer mux implementation with FSR
 * To be used on the openbionics robot hand sensors
 */

#include <ros.h>
#include <ros/time.h>
#include <th_messages/raw_barometer.h>
#include <std_msgs/UInt16MultiArray.h>

ros::NodeHandle nh;

#include <Wire.h>
#include <MPL115A2MUX.h>

#define NUM_SENSORS 1


MPL115A2MUX mpl115a2(NUM_SENSORS);
int fsrNum = 1;     //the FSR and 10K pulldown are connected to analog pins
int fsrReading;     // the analog reading from the FSR resistor divider

th_messages::raw_barometer rbaro;
ros::Publisher rbr_reads("raw_barometers", &rbaro);

std_msgs::UInt16MultiArray fsrs_msg;
ros::Publisher pub_fsrs("raw_fsrs", &fsrs_msg);

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
  delay(1000);
  digitalWrite(13, LOW);
  
  Serial.begin(115200);
  mpl115a2.begin();
  mpl115a2.upateSensorsLevel(); 
  //set flagHistoryExists=true;
  Serial.println("setup");
  
  // put your setup code here, to run once:
  nh.initNode();
  rbaro.header.frame_id = "baros_frames";
  nh.advertise(rbr_reads);
  fsrs_msg.data_length = fsrNum;
  nh.advertise(pub_fsrs);
  digitalWrite(13, HIGH);
  delay(1000);
}

void loop() {
  mpl115a2.upateSensorLevelID(0);
  mpl115a2.upateSensorsLevel();

  for(int i = 0; i < NUM_SENSORS; i++){
    //fsr readings
    uint16_t aux[fsrNum];
    for (int i=0; i<fsrNum; i++){
      fsrReading = analogRead(i);
      aux[i] = (int) fsrReading;
    }
    fsrs_msg.data = aux;

    Serial.print(mpl115a2.pressureHistory[i]); if(i<MAX_MUX-1)Serial.print(",");
    rbaro.sensor_id=i;
    rbaro.baro_level = mpl115a2.pressureHistory[i];
    rbaro.tempe = tempe;
    rbaro.header.seq = seq;
    rbaro.header.stamp = nh.now();

    pub_fsrs.publish( &fsrs_msg );
    rbr_reads.publish( &rbaro );
    
    nh.spinOnce();
  }
  seq++;
}
