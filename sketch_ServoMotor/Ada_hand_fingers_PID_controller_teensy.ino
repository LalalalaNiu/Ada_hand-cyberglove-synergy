// Source & info: www.HomoFaciens.de/technics-computer-arduino-uno_en_navion.htm
// ROS-specific parts were taken from: http://wiki.ros.org/rosserial_arduino/Tutorials/Servo%20Controller and other ROS tutorials
// This code follow the ROS StyleGuide for C++ (see: http://wiki.ros.org/CppStyleGuide)

#include <ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

//Number of motors
#define MOTOR_AMOUNT 5

/* The following associates the index value with a particular part of our hand:
 *  This information is for the current wiring of the right hand
 * 0 = Thumb
 * 1 = Index (works)
 * 2 = Middle
 * 3 = Ring
 * 4 = Pinky
 */
 
// Motors Pontentiometers Min and Max
#define MOTOR_0_MIN 50
#define MOTOR_0_MAX 950
#define MOTOR_1_MIN 50
#define MOTOR_1_MAX 950
#define MOTOR_2_MIN 50
#define MOTOR_2_MAX 950
#define MOTOR_3_MIN 50
#define MOTOR_3_MAX 950
#define MOTOR_4_MIN 50
#define MOTOR_4_MAX 950

ros::NodeHandle node_position;

int position_target[MOTOR_AMOUNT] = {511, 511, 511, 511, 511};      //Default to middle. Will be changed through ROS communication. Was a value read from a potentiometer, so 0-1023
int position_target_old[MOTOR_AMOUNT] = {511, 511, 511, 511, 511};  //SetPoint will not change for now
int16_t position_measured[MOTOR_AMOUNT] = {0, 0, 0, 0, 0};            	//Current value of the potentiometer
int position_measured_old[MOTOR_AMOUNT] = {0, 0, 0, 0, 0};        	//Last value of the potentiometer
int motor_speed[MOTOR_AMOUNT] = {30, 30, 30, 30, 30};             	//10 -> 255
int error_value[MOTOR_AMOUNT] = {0, 0, 0, 0, 0};            			//Stores difference between actual and wanted position
int time_diff[MOTOR_AMOUNT] = {0, 0, 0, 0, 0};              			//Stores time between two readings (for integral component)

int inverse_motor[MOTOR_AMOUNT] = {0, 0, 0, 0, 0};					//Inverse phase if we inverted the two inputs for the motor
bool motor_phase[MOTOR_AMOUNT] = {HIGH, HIGH, HIGH, HIGH, HIGH};

int position_min[MOTOR_AMOUNT] = {MOTOR_0_MIN, MOTOR_1_MIN, MOTOR_2_MIN, MOTOR_3_MIN, MOTOR_4_MIN};					//Set a minimum position for the motor
int position_max[MOTOR_AMOUNT] = {MOTOR_0_MAX, MOTOR_1_MAX, MOTOR_2_MAX, MOTOR_3_MAX, MOTOR_4_MAX};	//Set a maximum position for the motor

bool angle[MOTOR_AMOUNT] = {0, 0, 0, 0, 0};							//Need to convert from angle to pot value for some motors

//The max and min angles will not be used if the angle is set to false
//Set values in angle_max and angle_min, for indices where angles is 1. Currently, -90 is min and 90 is max
int angle_min[MOTOR_AMOUNT] = {-90, -90, -90, -90, -90};
int angle_max[MOTOR_AMOUNT] = {90, 90, 90, 90, 90};

//For testing: turn on only some of the motors at a time
//By default turn all the motors off. Then, we can use ROS to turn on specific motors
bool motor_on[MOTOR_AMOUNT] = {0, 0, 0, 0, 0};

//Format is: MIN -> MAX (Default)   

//Set Constants
float Kp = 1.0;//0.3;    //Proportional component:  0.0 -> 10.0 (0.3)
float Ki = 0.3;//4.0;    //Integral component:   0.0 -> 10.0 (0.3)
float Kd = 3.0;//4.0;    //Differential component:  0.0 -> 10.0 (4.0)
int deadzone = 10;             //10 -> 1000 (25)
float speed_min = 25;          //0 -> 255 (25)
float speed_max = 200;         //0 -> 255 (255)      (default to 200)
float soft_start = 0.7;          //0.00 -> 1.00 (0.30)  (1.00 = OFF)
int shutdown_deadlock = 4;     //0 -> 100 (4), 0 - OFF, Stops motor if blocked
int shoot_through_pause = 10;  //Prevent H bridge from shoot through whenever the direction pin is changed

//Set pin numbers
int8_t pin_phase[MOTOR_AMOUNT] = {19, 20, 21, 22, 23};   //Direction pin servo motor
int8_t pin_pwm[MOTOR_AMOUNT] = {3, 4, 5, 6, 9};     //PWM pin servo motor
int8_t pin_input[MOTOR_AMOUNT] = {14, 15, 16, 17, 18};   //Analog input servo sensor

//Setup to receive position from and send actual position to the computer
std_msgs::Int16MultiArray position_feedback;
//Publish the position feedback to a topic called "feedback"
ros::Publisher feedback("feedback", &position_feedback);

void positionCb(const std_msgs::Float32MultiArray& cmd_msg) {
      float position_received[MOTOR_AMOUNT] = {0, 0, 0, 0, 0}; 
      //sets wanted position using ROS messages, from 0-1023 (currently, this does *not* account for the physical limit of the motor)
      position_received[0] = cmd_msg.data[0];
      position_received[1] = cmd_msg.data[1];
      position_received[2] = cmd_msg.data[2];
      position_received[3] = cmd_msg.data[3];
      position_received[4] = cmd_msg.data[4];
      
      position_target[0] = (int)(position_received[0]*((MOTOR_0_MAX - MOTOR_0_MIN) + MOTOR_0_MIN));
      position_target[1] = (int)(position_received[1]*((MOTOR_1_MAX - MOTOR_1_MIN) + MOTOR_1_MIN));
      position_target[2] = (int)(position_received[2]*((MOTOR_2_MAX - MOTOR_2_MIN) + MOTOR_2_MIN));
      position_target[3] = (int)(position_received[3]*((MOTOR_3_MAX - MOTOR_3_MIN) + MOTOR_3_MIN));
      position_target[4] = (int)(position_received[4]*((MOTOR_4_MAX - MOTOR_4_MIN) + MOTOR_4_MIN));
}

//Subscribe to target position on a topic called "position"
ros::Subscriber<std_msgs::Float32MultiArray> position_sub("position", positionCb);

void motorsCb(const std_msgs::Int16MultiArray& cmd_msg2)
{
   //sets wanted position using ROS messages, from 0-1023 (currently, this does *not* account for the physical limit of the motor)
    motor_on[0] = cmd_msg2.data[0];
    motor_on[1] = cmd_msg2.data[1];
    motor_on[2] = cmd_msg2.data[2];
    motor_on[3] = cmd_msg2.data[3];
    motor_on[4] = cmd_msg2.data[4];
}

//Set which motors should be on on a topic called "enable_motors"
ros::Subscriber<std_msgs::Int16MultiArray> enable_motors_sub("enable_motors", motorsCb);

void parametersCb(const std_msgs::Float32MultiArray& cmd_msg3)
{
  // Set the parameters using ROS meesages: data: [Kp, Ki, Kd, deadzone, speed_min, speed_max, soft_start, shutdown_deadlock, shoot_through_pause]
  Kp = cmd_msg3.data[0];
  Ki = cmd_msg3.data[1];
  Kd = cmd_msg3.data[2];
  deadzone = (int)cmd_msg3.data[3];
  speed_min = cmd_msg3.data[4];
  speed_max = cmd_msg3.data[5];
  soft_start = cmd_msg3.data[6];
  shutdown_deadlock = (int)cmd_msg3.data[7];
  shoot_through_pause = (int)cmd_msg3.data[8];
}

// Set the parameters on a topic called "parameters"
ros::Subscriber<std_msgs::Float32MultiArray> parameters_sub("parameters", parametersCb);

void setup() { 
  //Initialize I/O pins
  for (int i=0; i<MOTOR_AMOUNT; i++) {
    pinMode(pin_phase[i], OUTPUT);
    pinMode(pin_pwm[i], OUTPUT);
    pinMode(pin_input[i], INPUT);
  // Not sure we need these with digitalwrite/analogwrite/analogread
  }
  position_feedback.data_length = MOTOR_AMOUNT;

  //Setup ROS communication
    //Receive wanted_position from computer
  node_position.initNode();
  node_position.subscribe(position_sub);
  delay(5);
  node_position.advertise(feedback);
  delay(5);
  node_position.subscribe(enable_motors_sub);
  delay(5);
  node_position.subscribe(parameters_sub);
  

  // If we want regular rotation, then check for direction of rotation, depending on the sign of the error value
  // This is necessary in case some of the motors are wired "backwards"
  for (int i=0; i<MOTOR_AMOUNT; i++)
    if(inverse_motor[i]==1) {
      //error_value[i] = -error_value[i];
      if(motor_phase[i]==true) {
        motor_phase[i] = false;
      } else {
        motor_phase[i] = true;
      }
  }
} 


void loop() {
  for (int j=0; j<MOTOR_AMOUNT; j++) {
    // Read the current/actual position from potentiometer (0->1023)
    position_measured[j] = analogRead(pin_input[j]);
    position_feedback.data = position_measured;
    
    // Map from an angle input to a potentiometer value we can use for control
    if(angle[j]==true) {
        position_target[j] = map(position_target[j], angle_min[j], angle_max[j], position_min[j], position_max[j]);
        position_measured[j] = map(position_measured[j], position_min[j], position_max[j], angle_min[j], angle_max[j]);
    }
    
    // If the target position is under the minimum or over the maximum that is physically
    // possible, don't go below the minimum or over the maximum
    if(position_target[j]<=position_min[j]) {
  	  position_target[j] = position_min[j];
    }
    if(position_target[j]>=position_max[j]) {
  	  position_target[j] = position_max[j];
    }
  
    // Error Value = Difference between wanted and actual value
    error_value[j] = position_target[j] - position_measured[j];
    
    // Implementation of the PID controller based on P, I and D
    motor_speed[j] = abs(error_value[j]) * Kp;                  //Proportional
    motor_speed[j] += time_diff[j] * Ki;                  //Integral
    motor_speed[j] += abs(position_target_old[j] - position_target[j]) * Kd; //Derivative
    
    // Prevent the servo from starting at full power, which could damage the motor/HBridge
    if(soft_start * time_diff[j] < 1) {
      motor_speed[j] = motor_speed[j] * (soft_start * time_diff[j]);
    }
    time_diff[j]++;
    
    // Prevent the motor from going under the minimum duty cycle
    if(motor_speed[j] < speed_min && motor_speed[j] > 0) {
      motor_speed[j] = speed_min;
    }
    
    // Prevent the motor from going above the maximum duty cycle
    if(motor_speed[j] > speed_max) {
      motor_speed[j] = speed_max;
    }
    
    // If the motor should be off, motor_speed[j]=0
    if(motor_speed[j] < 0) {
      motor_speed[j] = 0;
    }
        
    // If we are close enough to the wanted position, then stop the motor.
    if(abs(error_value[j]) < deadzone) {
      motor_speed[j] = 0;
      time_diff[j] = 0;
    }
  
    // Emergency_shutdown part: if the servomotor is emitting maximum power for more than a certain time,
    // and we're not moving the motor enough in a single loop, then set the duty cycle to 0 for a certain delay.
      if(abs(position_measured_old[j] - position_measured[j]) < shutdown_deadlock && motor_speed[j] == speed_max && time_diff[j] > 50) {
          analogWrite(pin_pwm[j], 0);
          delayMicroseconds(shoot_through_pause);
          digitalWrite(pin_phase[j], 0);
          //delay(1000);// TODO: Solve this delay with timers in a future version
          time_diff[j] = 0;
          
      } else {// If emergency_shutdown is not necessary...
          // Sets the motor speed to 0 if we don't want the motor to move
          if(motor_on[j]==false) {
            motor_speed[j] = 0;
          }
          
          if(error_value[j] > 0) {
            analogWrite(pin_pwm[j], 0);
            delayMicroseconds(shoot_through_pause);
            digitalWrite(pin_phase[j], abs(1-motor_phase[j]));
            delayMicroseconds(shoot_through_pause);
            analogWrite(pin_pwm[j], motor_speed[j]);
          }
          if(error_value[j] < 0) {
            analogWrite(pin_pwm[j], 0);
            delayMicroseconds(shoot_through_pause);
            digitalWrite(pin_phase[j], motor_phase[j]);
            delayMicroseconds(shoot_through_pause);
            analogWrite(pin_pwm[j], motor_speed[j]);
          }
      }
    
    //Setup the "previous values" for the I and D components on the next loop.
    position_target_old[j] = position_target[j];
    position_measured_old[j] = position_measured[j];
    
    
    //Map from a potentiometer value to an angle to compare feedback with the target position on the computer
    if(angle[j]==true) {
    	  position_measured[j] = map(position_measured[j], position_min[j], position_max[j], angle_min[j], angle_max[j]);
    }
  }
  feedback.publish(&position_feedback);
  //delay(10);
  node_position.spinOnce();
}
