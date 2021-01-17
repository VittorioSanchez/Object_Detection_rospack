#!/usr/bin/env python3.6
#CLEAR EVENTS AT THE BEGINNING

#Import threads classes
from Detection_Thread import Detection
from ROS_publisher_Thread import ROS_publisher

#Import Video init package
import detectnet_camera_custom_siec_noCV2 as detect


#Import global variables
import global_variables as glob

#Import ROS library
import rospy
from std_msgs.msg import UInt8

#MAIN PROGRAM
if __name__ == "__main__":
    
    #Create the topic "detection"and the publisher 
    #glob.pub.value = rospy.Publisher('detection', UInt8, queue_size=10)
    #Specify the nodes name
    rospy.init_node('talker', anonymous=True)
    print("ROS Topic created\n")
    
    #Init video input
    glob.input.value = detect.Video_Source_init("csi://0")

    #Create threads
    human_detection_thread = Detection("human")
    #hurdles_detection_thread = Detection("hurdles")
    ros_thread = ROS_publisher()

    human_detection_thread.daemon = True
    #hurdles_detection_thread.daemon = True
    ros_thread.daemon = True    
   
    #Run threads
    human_detection_thread.start()
    #hurdles_detection_thread.start()
    ros_thread.start()

    #Infinite loop because detection_thread has a while true
    human_detection_thread.join()
