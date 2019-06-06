
# '''
#     This node would transfer the /leapmotion/hand data to calculate the parameters for 
#         supervising the synergy of ada_hand.
    
# '''

# from std_msgs.msg import String
# import serial
# import rospy
# from sensor_msgs.msg import JointState
# import sys

# def LeapCalculator(marker_msg):
# # set the actuator condition: 0~255, 64 start to vibrate
    
    
#     Leap_pub = rospy.Publisher('/leaptrans/synergy', Leapmarkers, queue_size=2)



#     # TODO: Calibrate the values to be published in the topic /calibrated/joint_states or 
#     #  separate the data to what we only use: finger (thumb 2) and the wrist.
    
#     # jointstate_raw_msg.position[1] 
#     # jointstate_raw_msg.position[2]
#     # jointstate_raw_msg.position[5] 
#     # jointstate_raw_msg.position[7]
#     # jointstate_raw_msg.position[10]
#     # jointstate_raw_msg.position[13]

#     #inherit the Publisher pub_name
#     leap_msg = Leapmarkers
#     # Set the size of jointstate_raw_msg.position[] (18 sensors)
    

#     #pubulish the actuator control signal:
#     Leap_pub.publish(leap_msg)



# if __name__ == '__main__':

#     rospy.init_node('leaptrans',anonymous=True)
#     #subscribe to a topic using rospy.Subscriber class
#     actuator_sub = rospy.Subscriber('/leap_motion/hands', marker_msg, LeapCalculator)
#     rospy.spin()