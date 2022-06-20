#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
# Using a String Message Type

def faxer():
    
    # Let ROS know that this is a Node
    rospy.init_node('faxer', anonymous=True) #With anonymous you can spawn many 'faxers' with no conflicts
    
    # Make a Topic
    pub = rospy.Publisher('fax_line', String, queue_size=10) #Only store the 10 latest String messages
    
    # Set the rate at which messages are published
    rate = rospy.Rate(10) # Hz
    
    while not rospy.is_shutdown(): # K eep running until ROS tells you to stop
        message = input("Enter your input: ")
        pub.publish(message)
        rate.sleep() # enables the loop to run at the Hz level you defined

if __name__ == "__main__":
    
    # try-catch block makes it easier for ROS to figure out what went wrong in case of error
    try:
        faxer()
    except rospy.ROSInterruptException:
        pass




