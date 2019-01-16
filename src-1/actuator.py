#!/usr/bin/env python
'''
    This node would use the cybertouch_raw_msg data to actuate the actuators.
    (would realated on the rosbot hand feed back)
'''

from std_msgs.msg import String
from sensor_msgs.msg import JointState
import serial
import rospy
import sys

def actuator(jointstate_raw_msg):
# set the actuator condition: 0~255, 64 start to vibrate
    actuator_pub = rospy.Publisher('/cybertouch/actuator_states', String, queue_size=2)


  #  print (jointstate_raw_msg.position[2])
    
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
        a1 = ' ' + str(a1)
    else:
        a1 =' 0'    
    
    if 0.45 <= jointstate_raw_msg.position[5] <= 0.7:
        a2 = jointstate_raw_msg.position[5]*760 - 277
        a2 = int(a2)
        a2 = ' ' + str(a2)
    else:
        a2 =' 0'

    if 0.45 <= jointstate_raw_msg.position[7] <= 0.7:
        a3 = jointstate_raw_msg.position[7]*760 - 277
        a3 = int(a3)
        a3 = ' ' + str(a3)
    else:
        a3 =' 0'
 
    if 0.45 <= jointstate_raw_msg.position[10] <= 0.7:
        a4 = jointstate_raw_msg.position[10]*760 - 277
        a4 = int(a4)
        a4 = ' ' + str(a4)
    else:
        a4 =' 0'

    if 0.45 <= jointstate_raw_msg.position[13] <= 0.7:
        a5 = jointstate_raw_msg.position[13]*760 - 277
        a5 = int(a5)
        a5 = ' ' + str(a5)
    else:
        a5 =' 0'

    actuator_msg = 'a 255' + a1 + a2 + a3 + a4 + a5 + ' 0'
    print actuator_msg

    #pubulish the actuator control signal:
    actuator_pub.publish(actuator_msg)



if __name__ == '__main__':

    rospy.init_node('actuator',anonymous=True)
    #subscribe to a topic using rospy.Subscriber class
    actuator_sub = rospy.Subscriber('/cybertouch/raw/joint_states', JointState, actuator)
    rospy.spin()
 

 