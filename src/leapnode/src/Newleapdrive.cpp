#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"
#include <Human.h>
#include <Hand.h>
#include <Finger.h>
#include <Bone.h>
#include <Leap.h> 

#include <iostream>


static ros::Publisher* marker_pub;

const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};




void leaptrans(const leap_motion::Human::ConstPtr& human, leap_motion::Hand hand, std::string hand_ns, uint8_t hand_id)
{

    //define the output parameters.
    sensor_msgs::JointState marks;
    


    leap_motion::Finger finger;

    for(unsigned int j = 0; j < hand.finger_list.size(); j++)
    {
        finger = hand.finger_list[j];
        std::string n_f = fingerNames[finger.type];
        
    
      

        leap_motion::Bone bone;

        for(unsigned int k = 1; k < finger.bone_list.size(); k++)
        {
            bone = finger.bone_list[k];
        
            //std::cout << ns_finger << std::endl;
            std::string nn_f;
            nn_f = std::to_string (k) + " " + n_f + "x " + n_f +"y " + n_f +"z";

            std::cout << nn_f << std::endl;
            std::cout << bone.bone_start << std::endl;

            //sequence from palm to fingertip
            marks.name.push_back(nn_f);
            marks.position.push_back(bone.bone_start.position.x);
            marks.position.push_back(bone.bone_start.position.y);
            marks.position.push_back(bone.bone_start.position.z);
         
        }
    
    }

    
    marks.name.push_back("0 palm_center");
    marks.position.push_back(hand.palm_center.x);
    marks.position.push_back(hand.palm_center.y);
    marks.position.push_back(hand.palm_center.z);

    marker_pub->publish(marks);

}

/*!
 * \brief Called when a new Human.msg is received.
 * 
 * \param human    Pointer to thr received Human.msg
 */
void frameCallback(const leap_motion::Human::ConstPtr& human){

    
    leap_motion::Hand hand;

    if( human -> right_hand.is_present )
    {
        std::cout << "please use the left hand" << std::endl;
            }
    
    if( human -> left_hand.is_present )
    {
        hand = human -> left_hand;
        

        leaptrans(human, hand, "Left", 1);
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "leapnode");
    ros::NodeHandle nh("leapnode");
    bool enable_filter;

    //setup the subscriber

    nh.getParam("/enable_filter", enable_filter);
    ros::Subscriber leap_sub;

    leap_sub = nh.subscribe<leap_motion::Human>("/leap_motion/leap_device", 1, frameCallback);
    if(enable_filter)
    {
        leap_sub = nh.subscribe<leap_motion::Human>("/leap_motion/leap_filtered", 1, frameCallback);
    }

    ROS_INFO("enable_filter: %s", enable_filter ? "true" : "false");
    
    //define the publish topic
    ros::Publisher m_pub = nh.advertise<sensor_msgs::JointState>("handinfo_array", 100);
    marker_pub = &m_pub;

    ros::spin();
    return 0;
}

