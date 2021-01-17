import time
import numpy as np
import os
import sys

#LIBRARY FOR MULTI-THREADING
import threading
from threading import Thread, Lock

#ROS library
import rospy
from std_msgs.msg import UInt8

#Import global variables
import global_variables as glob

#Class for publishing messages on ROS
class ROS_publisher(Thread):

    def __init__(self):
        Thread.__init__(self) 

    def run(self):
        """
        #Create the topic "detection"and the publisher 
        pub = rospy.Publisher('detection', UInt8, queue_size=10)
        #Specify the nodes name
        rospy.init_node('talker', anonymous=True)
        """

        #Specify the rate (we do note need it here)
        rate = rospy.Rate(25) # 25hz

        current_flag_value = 0

        while not rospy.is_shutdown():

            #We update the value of detection_flag
            #print("\nGET FLAG VALUE\n") #Debug
            glob.detection_flag.MUT.acquire()
            current_flag_value = glob.detection_flag.value
            glob.detection_flag.MUT.release()

            #print("\nREACT TO DETECTION IF SOMETHING IS DETECTED\n") #Debug
            #print('ROS current flag value: ', current_flag_value)
            if (current_flag_value == 1): #if detection threads detect something we stop ROS periodic sending for 1sec
                #print("\nSOMETHING IS DETECTED\n") #Debug
                #print("\nCLEAR FLAG\n") #Debug                
                glob.detection_flag.MUT.acquire()
                glob.detection_flag.value = 0
                glob.detection_flag.MUT.release()

                #Periodic sending out for 1 sec
                #print("\nSENDIND DEACTIVATED\n") #Debug
                time.sleep(2)
                #print("\nSENDIND REACTIVATED\n") #Debug

            else:
                #print("\nSEND RAS MESSAGE ON ROS TOPIC\n") #Debug
                glob.pub.MUT.acquire()
                glob.pub.value.publish(0)
                glob.pub.MUT.release()
                
            rate.sleep() #Only if we uses fixed rate

#MAIN PROGRAM
if __name__ == "__main__":

    #Create the topic "detection"and the publisher
    print("Init ROS topic\n") #Debug
    glob.pub.value = rospy.Publisher('detection', UInt8, queue_size=10)
    #Specify the nodes name
    rospy.init_node('talker', anonymous=True)

    #Create thread
    ros_thread = ROS_publisher()

    #Start thread
    ros_thread.start()

    while True:
        pass
