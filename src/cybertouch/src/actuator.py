#!/usr/bin/env python
'''
    This node would use the cybertouch_raw_msg data to actuate the actuators.
    (would realated on the rosbot hand feed back)
'''

from std_msgs.msg import String
import serial
import rospy
from sensor_msgs.msg import JointState

import sys

def actuator(jointstate_raw_msg):
# set the actuator condition: 0~255, 64 start to vibrate
    actuator_pub = rospy.Publisher('/cybertouch/actuator_states', String, queue_size=2)


  #  print (jointstate_raw_msg.position[2])
    # TODO: Calibrate the values to be published in the topic /calibrated/joint_states or 
    #  separate the data to what we only use: finger (thumb 2) and the wrist.
    
    # jointstate_raw_msg.position[1] 
    # jointstate_raw_msg.position[2]
    # jointstate_raw_msg.position[5] 
    # jointstate_raw_msg.position[7]
    # jointstate_raw_msg.position[10]
    # jointstate_raw_msg.position[13]

    #inherit the Publisher pub_name
    actuator_msg = String
    # Set the size of jointstate_raw_msg.position[] (18 sensors)
    if 0.45 <= jointstate_raw_msg.position[1] <= 0.7:
        a1 = jointstate_raw_msg.position[1]*760 - 277
        a1 = int(a1)
        a1 = ' ' + "%03d" % a1
    else:
        a1 =' 000'    
    
    if 0.45 <= jointstate_raw_msg.position[5] <= 0.7:
        a2 = jointstate_raw_msg.position[5]*760 - 277
        a2 = int(a2)
        a2 = ' ' + "%03d" % a2
    else:
        a2 =' 000'

    if 0.45 <= jointstate_raw_msg.position[7] <= 0.7:
        a3 = jointstate_raw_msg.position[7]*760 - 277
        a3 = int(a3)
        a3 = ' ' + "%03d" % a3
    else:
        a3 =' 000'
 
    if 0.45 <= jointstate_raw_msg.position[10] <= 0.7:
        a4 = jointstate_raw_msg.position[10]*760 - 277
        a4 = int(a4)
        a4 = ' ' + "%03d" % a4
    else:
        a4 =' 000'

    if 0.45 <= jointstate_raw_msg.position[13] <= 0.7:
        a5 = jointstate_raw_msg.position[13]*760 - 277
        a5 = int(a5)
        a5 = ' ' + "%03d" % a5
    else:
        a5 =' 000'

    actuator_msg = 'a 6' + ' 0' + a1 + ' 1' + a2 + ' 2' + a3 + ' 3' + a4 + ' 4' + a5 + ' 5' + ' 000 \r'
    print actuator_msg

    #pubulish the actuator control signal:
    actuator_pub.publish(actuator_msg)



if __name__ == '__main__':

    rospy.init_node('actuator',anonymous=True)
    #subscribe to a topic using rospy.Subscriber class
    actuator_sub = rospy.Subscriber('/cybertouch/raw/joint_states', JointState, actuator)
    rospy.spin()