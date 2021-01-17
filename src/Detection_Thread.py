# MIT License
# Copyright (c) 2019 JetsonHacks
# See license
# Using a CSI camera (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit using OpenCV
# Drivers for the camera and OpenCV are included in the base image

import time
import detectnet_camera_custom_siec_noCV2 as detect
import numpy as np
import os
import sys

import random
import jetson.utils
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import Image
from ros_numpy import msgify


#LIBRARY FOR MULTI-THREADING
import threading
from threading import Thread, Lock

#ROS Library
import rospy
from std_msgs.msg import UInt8

#GLOBAL VARIABLES
import global_variables as glob

#function that return the right number regarding what was detected for Hurdles CNN
def hurdles_detection_to_ROS_number(class_number):
    if class_number == 1: #Ball
        return 3
    elif class_number == 2: #Bicycle
        return 4
    elif class_number == 3: #Bus
        return 5
    elif class_number == 4: #Car
        return 6
    elif class_number == 5: #Cat
        return 7
    elif class_number == 6: #Dog
        return 8
    elif class_number == 7: #Motorcycle
        return 9
    else:
        return 0
            
#function that return the right number regarding what was detected for the human net
def human_detection_to_ROS_number(class_number):
    if class_number == 0: #Person
        return 1
    elif class_number == 2: #Baggage
        return 2
    else:
        return 0
            


#Class for CNNs initializations and inferences
class Detection(Thread):

    def __init__(self, target, measure_time=False):
        Thread.__init__(self)
        self.target = target
        self.measure_time = measure_time
        if target == 'human':
            #We init detection CNN
            self.net = detect.Multiped_Init()
            self.detection_to_ROS_number = human_detection_to_ROS_number
            print("HUMAN INIT WENT WELL\n") #Debug
        elif target == 'hurdles':
            #We init detection CNN
            self.net = detect.Hurdles_Init()
            self.detection_to_ROS_number = hurdles_detection_to_ROS_number
            print("HURDLES INIT WENT WELL\n")
        time.sleep(2) #Just to have time to see the debug message

    def run(self):
        
        while True:
            #We process detection
            detections, img = detect.Imageprocessing(self.net, "./treated_current_pic_"+self.target+".png")
            
            #Send image to ROS topic
            glob.pub.MUT.acquire()
            glob.pub.image.publish(msgify(Image, jetson.utils.cudaToNumpy(img), "rgb8"))
            glob.pub.MUT.release()
            
            #if we detect something set flag to block ROS periodic sending, ROS thread clears it
            if (len(detections) != 0):
                print('size detections: ', len(detections))
                glob.detection_flag.MUT.acquire()
                glob.detection_flag.value = 1
                glob.detection_flag.MUT.release()

            glob.detection_flag.MUT.acquire() #Debug
            print("FLAG = ", glob.detection_flag.value) #Debug
            glob.detection_flag.MUT.release() #Debug

            #Enumerate all the detections for hurdles detection
            for detection in detections:
                print("\nWe detected a " + self.net.GetClassDesc(detection.ClassID) + "\n")
                glob.pub.MUT.acquire()
                glob.pub.value.publish(self.detection_to_ROS_number(detection.ClassID))
                glob.pub.left.publish(detection.Left)
                glob.pub.right.publish(detection.Right)
                glob.pub.top.publish(detection.Top)
                glob.pub.bottom.publish(detection.Bottom)
                glob.pub.MUT.release()

        
#MAIN PROGRAM
if __name__ == "__main__":
    
    # Création des threads
    detection_thread = Detection()

    # Lancement des threads
    detection_thread.start()
    detection_thread.daemon = True

    # Attend que les threads se terminent
    detection_thread.join()

    
