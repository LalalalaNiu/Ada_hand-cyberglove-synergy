#include "ros/ros.h"

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

    // marker_array->markers.push_back(createPalmPosition(human, hand_ns, 55, hand.palm_center));
    // marker_array->markers.push_back(createHandOutline(human, hand_ns, hand_id, hand) );

    leap_motion::Finger finger;
    for(unsigned int j = 0; j < hand.finger_list.size(); j++)
    {
        finger = hand.finger_list[j];
        std::string ns_finger = fingerNames[finger.type];
        
        std::cout << ns_finger << std::endl;

        unsigned int id_offset = finger.bone_list.size();
        
        leap_motion::Bone bone;
        for(unsigned int k = 1; k < finger.bone_list.size(); k++)
        {
            bone = finger.bone_list[k];
            // marker_array->markers.push_back(createFingerLines(human, hand_ns, ns_finger, k, bone));
            // marker_array->markers.push_back(createJointMarker(human, hand_ns, ns_finger, k + id_offset, bone.bone_start));
            
            // The circle at the very bottom of the pinky
            if(finger.type == Leap::Finger::Type::TYPE_PINKY)
            {
                leap_motion::Bone temp_bone = finger.bone_list[0];
                // marker_array->markers.push_back(createJointMarker(human, hand_ns, ns_finger,
                //     k + id_offset+1, temp_bone.bone_start));    
            }
            // Fingertip circles
            if(k == Leap::Bone::Type::TYPE_DISTAL)
            {
               // marker_array->markers.push_back(createJointMarker(human, hand_ns, ns_finger,
                //    k + id_offset + 2, bone.bone_end));
            }
        }
    }
}

/*!
 * \brief Called when a new Human.msg is received.
 * 
 * \param human    Pointer to thr received Human.msg
 */
void frameCallback(const leap_motion::Human::ConstPtr& human){

    std_msgs::String marker_array;
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
    
    marker_pub->publish(marker_array);
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
    ros::Publisher m_pub = nh.advertise<std_msgs::String>("handinfo_array", 100);
    marker_pub = &m_pub;

    ros::spin();
    return 0;
}

