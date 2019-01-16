#!/usr/bin/env python

'''  
This node extracts data from the cyberglove through serial connection and publishes them in the topics: 

- /cybertouch/raw/joint_states
'''
 
# TODO: Calibrate the values to be published in the topic /calibrated/joint_states or 
# take it out as we use only the topic /raw/joint_states to calculate the fingers' positions

import serial
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import sys
from time import sleep

# Open serial port

port_name = "/dev/ttyUSB0"

ser = serial.Serial(port_name, 115200) 
ser.flush()	

ser.write("t 11520 1 \r")
ser.readline(12)
ser.write("S")
ActD = ""

def gloveSerial():
	

    # ser.write('N 22\r') # Set the number of sensors
    # ser.readline()

    # ser.write('T 1152 1\r') # Set frequency to 100 Hz
    # ser.readline()

    
    # Publishes raw JointState messages
    cyberglove_raw_pub = rospy.Publisher('/cybertouch/raw/joint_states', JointState, queue_size=2)
    rospy.init_node('gloveSerial', anonymous=True)

    rate = rospy.Rate(100) # 100 Hz

    # initialises joint names (the order is important)
     
    #inherit the Publisher pub_name
    jointstate_raw_msg = JointState()	
    #set the name
    jointstate_raw_msg.name.append("G_ThumbRotate")
    jointstate_raw_msg.name.append("G_ThumbMPJ")
    jointstate_raw_msg.name.append("G_ThumbIJ")
    jointstate_raw_msg.name.append("G_ThumbAb")

    jointstate_raw_msg.name.append("G_IndexMPJ")
    jointstate_raw_msg.name.append("G_IndexIJ")

    
    jointstate_raw_msg.name.append("G_MiddleMPJ")
    jointstate_raw_msg.name.append("G_MiddleIJ")
    jointstate_raw_msg.name.append("G_MiddleIndexAb")
    
    jointstate_raw_msg.name.append("G_RingMPJ")
    jointstate_raw_msg.name.append("G_RingIJ")
    jointstate_raw_msg.name.append("G_RingMiddleAb")
    
    jointstate_raw_msg.name.append("G_PinkieMPJ")
    jointstate_raw_msg.name.append("G_PinkieIJ")
    jointstate_raw_msg.name.append("G_PinkieRingAb")
    
    jointstate_raw_msg.name.append("G_PalmArch")
    jointstate_raw_msg.name.append("G_WristPitch")
    jointstate_raw_msg.name.append("G_WristYaw")


    # Set the size of jointstate_raw_msg.position[] (18 sensors)
    for i in range (0, 18):
        jointstate_raw_msg.position.append(0)
  

    while not rospy.is_shutdown():
        
        while True:
            if ord(ser.read()) == 0:
                msg_line = ser.readline(19) # read whole line of the binary output.
                break
        
        msgline = msg_line[1:19]

        i = 0

        for c in msgline: # Sensors s1, s2,..., s18
            # The values sent by the glove are in the range [1;255]
            # We convert them to float in the range [0;1]

            jointstate_raw_msg.position[i] = (ord(c) - 1.0)/ 254.0
            i = i + 1
            
        #publish the data:
        cyberglove_raw_pub.publish(jointstate_raw_msg)
        
        rate.sleep()
        
        rospy.Subscriber('/cybertouch/actuator_states', String, gloveActuator)

        ser.write(ActD)
        ser.readline(43)
        #print ActuatorD
       
    ser.write("\r")
        

  
def gloveActuator(actuator_msg):

    global ActD

    ActD = actuator_msg.data
   
  
    # ser.write(str(actuator_msg.data))
    
    # ser.write("a 2 3 200 5 200 \r")
    # # sleep(1)
    # # ser.write("\r")

   
  

  
    

if __name__ == '__main__':
    
    if len(sys.argv) < 2:
        print("usage: Tserial.py serial_port")
    else:
        port_name = sys.argv[1]
      #  print("Gathering data from Cybertouch...");
        try:
            gloveSerial()
            
        
        except rospy.ROSInterruptException:
            pass
    
   