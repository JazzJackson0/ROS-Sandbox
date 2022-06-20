#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
#Using a String Message Type

#ROS Likes when you have a specific function to deal with incomming messaages
def fax_handler(data):
    
	#The ROS version of print(). The ROS version will also write the data to a file.
    rospy.loginfo(data.data) #data.data will end up being String.data


def printer():
    
	rospy.init_node('printer', anonymous=True)
    
	#Listen to the Topic
    rospy.Subscriber('fax_line', String, fax_handler) #Last Argument: The function that should handle the incomming messages
    #Keep the ssubscription going. So program wont end after taking 1 peice of data
        #But it also wont waste CPU space waiting.
    
	rospy.spin()

if __name__ == "__main__":
    printer()





