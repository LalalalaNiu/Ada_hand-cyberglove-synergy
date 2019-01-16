import serial
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import sys
from time import sleep


port_name = "/dev/ttyUSB0"

ser = serial.Serial(port_name, 115200) 
ser.flush()	


# s= "a 6 0 200 1 200 2 200 3 200 4 200  200 6 200 \r"

# ser.write("t 11520 1 \r")
# s = ser.readline(12)
# for c in s:
#     print ord(c) + '^^^^^'

# ser.write("a 255 000 000 000 000 000 000 \r")

# s = ser.readline(33)
# for c in s:
#     print ord(c)

# ser.write("S")
# ser.read()
# i = 20

# while i !=0:
    
#     while True:
#         if ord(ser.read()) == 0:
#             line = ser.readline(19)
#             line = line[1:19]
#     # EVALUATE THE LENGTH
#             print line
#             break
#     # line = line[0:20]
    

#     # for c in line: 
#     #     print ord(c) 
    
#     # if i%5 == 0:
#     #     # ser.write("a 6 0 090 1 050 2 090 3 100 4 090 5 090 \r")
#     #     ser.write("t 3840 1 \r")
#     #     s = ser.readline(11)
#     #     # s = ser.readline(61)
#     #     ser.read()
       
#     #     print '^^^^^^^^' + s + '   '
#     #     print i
#     i = i-1

# ser.write("\r")
s = ''

def test(actuator_msg):
    global s

    s = actuator_msg.data
    
    

print s

if __name__ == '__main__':
    
    rospy.init_node('Test',anonymous=True)
    #subscribe to a topic using rospy.Subscriber class
    test_sub = rospy.Subscriber('/cybertouch/actuator_states', String, test)
   
    rospy.spin()
 
