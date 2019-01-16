/* 
* This node remaps the data from cyberglove to positions sent to Ada hand  
* It gets the cyberglove's data by subscribing the topic "/cyberglove/raw/joint_states"
* It calculates the finger positions through weighted average of the finger joints values
* It publishes the finger positions in the topic 'position'   

* For better results:
* Thumb and Pinkie positions - Weighted average
* Other fingers positions - Arithmetic mean
*/

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <algorithm>

enum Fingers {Thumb, Index, Middle, Ring, Pinkie, Wrist};

// Only the joints that will be used 
const std::vector<std::string> ThumbJoints = {"G_ThumbRotate", "G_ThumbMPJ", "G_ThumbIJ"};
const std::vector<std::string> IndexJoints = {"G_IndexDIJ", "G_IndexMPJ", "G_IndexPIJ"}; 
const std::vector<std::string> MiddleJoints = {"G_MiddleDIJ", "G_MiddleMPJ", "G_MiddlePIJ"};
const std::vector<std::string> RingJoints = {"G_RingDIJ", "G_RingMPJ", "G_RingPIJ"};
const std::vector<std::string> PinkieJoints = {"G_PinkieDIJ", "G_PinkieMPJ", "G_PinkiePIJ"};
const std::vector<std::string> WristJoints = {"G_WristPitch", "G_WristYaw"};

// Min and max values will be used to calibrate the glove
// Set the min and max values by comparing the current values received form the glove to the lastest min and max
double minValueThumbJoints[]  = {1.0, 1.0, 1.0};
double minValueIndexJoints[]  = {1.0, 1.0, 1.0};
double minValueMiddleJoints[] = {1.0, 1.0, 1.0};
double minValueRingJoints[]   = {1.0, 1.0, 1.0};
double minValuePinkieJoints[] = {1.0, 1.0, 1.0};
double minValueWristJoints[]  = {1.0, 1.0};

double maxValueThumbJoints[]  = {0, 0, 0};
double maxValueIndexJoints[]  = {0, 0, 0};
double maxValueMiddleJoints[] = {0, 0, 0};
double maxValueRingJoints[]   = {0, 0, 0};
double maxValuePinkieJoints[] = {0, 0, 0};
double maxValueWristJoints[]  = {0, 0};


// Wheight of each joint to calculate the position to be sent
// Sum of the wheights has to be 1
double WThumb[3]  = {0, 1, 0}; // Weighted average for thumb position
double WIndex[3]  = {0, 1, 0};
double WMiddle[3] = {0, 1, 0};
double WRing[3]   = {0, 1, 0};
double WPinkie[3] = {0, 1, 0}; // Weighted average for pinkie position
double WWrist[2]  = {0, 0};


// Convert the joints' values in the range [0;1] 
double divr = 0; // divr = max - min
double divr_aux = 0;
double min = 0;
double max = 0;
double jointValue = 0;
double jointPosition = 0; // jointPosition = (jointValue - min)/divr
double pos[] = {0, 0, 0, 0, 0, 0};	// pos = Weighted average(jointPosition) or Arithmetic mean (jointPosition)
std_msgs::Float32MultiArray position; // position = pos
bool next = false;


// Callback function for the topic '/cyberglove/raw/joint_states'
void setPositionCb(const sensor_msgs::JointState::ConstPtr& msg) {
	if (next)
		return;

	for(int k=0; k<6; k++)
		pos[k] = 0;

	// Thumb
	for (int i=0; i<3; i++) { // 3 joints used for the Thumb
		long j = std::find(msg->name.begin(), msg->name.end(), ThumbJoints[i]) - msg->name.begin();
		jointValue = msg->position[j];

		// Calibration
		if (jointValue < minValueThumbJoints[i])
			minValueThumbJoints[i] = jointValue;
		if (jointValue > maxValueThumbJoints[i])
			maxValueThumbJoints[i] = jointValue;

		divr = maxValueThumbJoints[i] - minValueThumbJoints[i];
		// Just to avoid 0/0 in the first iteration
		if (divr == 0)
			divr = 1;

		min = minValueThumbJoints[i] + 0.1*divr;
		max = maxValueThumbJoints[i] - 0.1*divr;

		divr_aux = max - min;

		jointPosition = (jointValue - min)/divr_aux;

		// Make sure it's in the range [0;1]
		if (jointPosition > 1)
			jointPosition = 1;
		else
		if (jointPosition < 0)
			jointPosition = 0;

		pos[0]+=(jointPosition * WThumb[i]);
		//pos[0]+=jointPosition;
	}
	//pos[0]/=3;	// If arithmetic mean (no wheights)

	// Index
	for (int i=0; i<3; i++) {
		long j = std::find(msg->name.begin(), msg->name.end(), IndexJoints[i]) - msg->name.begin();
		jointValue = msg->position[j];

		// Calibration
		if (jointValue < minValueIndexJoints[i])
			minValueIndexJoints[i] = jointValue;
		if (jointValue > maxValueIndexJoints[i])
			maxValueIndexJoints[i] = jointValue;

		divr = maxValueIndexJoints[i] - minValueIndexJoints[i];
		// Just to avoid 0/0 in the first iteration
		if (divr == 0)
			divr = 1;

		min = minValueIndexJoints[i] + 0.1*divr;
		max = maxValueIndexJoints[i] - 0.1*divr;

		divr_aux = max - min;

		jointPosition = (jointValue - min)/divr_aux;

		// Make sure it's in the range [0;1]
		if (jointPosition > 1)
			jointPosition = 1;
		else
		if (jointPosition < 0)
			jointPosition = 0;

//		pos[1]+=(jointPosition * WIndex[i]);
		pos[1]+=jointPosition;
	}
	pos[1]/=3;

	// Middle
	for (int i=0; i<3; i++) {
		long j = std::find(msg->name.begin(), msg->name.end(), MiddleJoints[i]) - msg->name.begin();
		jointValue = msg->position[j];

		// Calibration
		if (jointValue < minValueMiddleJoints[i])
			minValueMiddleJoints[i] = jointValue;
		if (jointValue > maxValueMiddleJoints[i])
			maxValueMiddleJoints[i] = jointValue;

		divr = maxValueMiddleJoints[i] - minValueMiddleJoints[i];
		// Just to avoid 0/0 in the first iteration
		if (divr == 0)
			divr = 1;

		min = minValueMiddleJoints[i] + 0.1*divr;
		max = maxValueMiddleJoints[i] - 0.1*divr;

		divr_aux = max - min;

		jointPosition = (jointValue - min)/divr_aux;

		// Make sure it's in the range [0;1]
		if (jointPosition > 1)
			jointPosition = 1;
		else
		if (jointPosition < 0)
			jointPosition = 0;

//		pos[2]+=(jointPosition * WMiddle[i]);
		pos[2]+=jointPosition;
	}
	pos[2]/=3;

	// Ring
	for (int i=0; i<3; i++) {
		long j = std::find(msg->name.begin(), msg->name.end(), RingJoints[i]) - msg->name.begin();
		jointValue = msg->position[j];

		// Calibration
		if (jointValue < minValueRingJoints[i])
			minValueRingJoints[i] = jointValue;
		if (jointValue > maxValueRingJoints[i])
			maxValueRingJoints[i] = jointValue;

		divr = maxValueRingJoints[i] - minValueRingJoints[i];
		// Just to avoid 0/0 in the first iteration
		if (divr == 0)
			divr = 1;

		min = minValueRingJoints[i] + 0.1*divr;
		max = maxValueRingJoints[i] - 0.1*divr;

		divr_aux = max - min;

		jointPosition = (jointValue - min)/divr_aux;

		// Make sure it's in the range [0;1]
		if (jointPosition > 1)
			jointPosition = 1;
		else
		if (jointPosition < 0)
			jointPosition = 0;

//		pos[3]+=(jointPosition * WRing[i]);
		pos[3]+=jointPosition;
	}
	pos[3]/=3;

	// Pinkie
	for (int i=0; i<3; i++) {
		long j = std::find(msg->name.begin(), msg->name.end(), PinkieJoints[i]) - msg->name.begin();
		jointValue = msg->position[j];

		// Calibration
		if (jointValue < minValuePinkieJoints[i])
			minValuePinkieJoints[i] = jointValue;
		if (jointValue > maxValuePinkieJoints[i])
			maxValuePinkieJoints[i] = jointValue;

		divr = maxValuePinkieJoints[i] - minValuePinkieJoints[i];
		// Just to avoid 0/0 in the first iteration
		if (divr == 0)
			divr = 1;

		min = minValuePinkieJoints[i] + 0.1*divr;
		max = maxValuePinkieJoints[i] - 0.1*divr;

		divr_aux = max - min;

		jointPosition = (jointValue - min)/divr_aux;

		// Make sure it's in the range [0;1]
		if (jointPosition > 1)
			jointPosition = 1;
		else
		if (jointPosition < 0)
			jointPosition = 0;

		pos[4]+=(jointPosition * WPinkie[i]);
//		pos[4]+=jointPosition;
	}
	//pos[4]/=3;

	// Wrist
	for (int i=0; i<2; i++) { // Only 2 joints
		long j = std::find(msg->name.begin(), msg->name.end(), WristJoints[i]) - msg->name.begin();
		jointValue = msg->position[j];

		// Calibration
		if (jointValue < minValueWristJoints[i])
			minValueWristJoints[i] = jointValue;
		if (jointValue > maxValueWristJoints[i])
			maxValueWristJoints[i] = jointValue;

		divr = maxValueWristJoints[i] - minValueWristJoints[i];
		// Just to avoid 0/0 in the first iteration
		if (divr == 0)
			divr = 1;

		min = minValueWristJoints[i] + 0.1*divr;
		max = maxValueWristJoints[i] - 0.1*divr;

		divr_aux = max - min;

		jointPosition = (jointValue - min)/divr_aux;

		// Make sure it's in the range [0;1]
		if (jointPosition > 1)
			jointPosition = 1;
		else
		if (jointPosition < 0)
			jointPosition = 0;

//		pos[5]+=(jointPosition * WWrist[i]);
		pos[5]+=jointPosition;
	}
	pos[5]/=2;

	next = true;
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "cyberglove_remap");
	ros::NodeHandle nh;

	
	position.data.resize(6); // 5 Fingers + Wrist

	ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("/position", 32); // Publish the position
	ros::Subscriber sub = nh.subscribe("/cybertouch/raw/joint_states", 32, setPositionCb); // Calculate position from the raw/joint_states' values

	ROS_INFO("Starting mapping...");
	ROS_INFO("Open and close your hand to calibrate the glove.");	
	
	while(ros::ok()) {
		if (next) {
			for(int i=0; i<6; i++) {

//				position.data[i] = 1 - pos[i];
                position.data[i] = 1 - pos[i];
			}
			pub.publish(position);			
			next = false;
		}
		ros::spinOnce();
	}	
	ros::shutdown();

	ROS_INFO("Bye.");

	return 0;
}

