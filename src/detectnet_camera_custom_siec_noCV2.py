#!/usr/bin/python3
#
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#

import jetson.inference
import jetson.utils

import sys
import time

#GLOBAL VARIABLES
import Object_Detection_rospack.global_variables as glob

def Multiped_Init():
        #Variables
        network = "multiped"
        threshold = 0.5
        arguments = []
        Overlay = "box,labels,conf"

        # load the object detection network
        net = jetson.inference.detectNet(network, arguments, threshold)

        return net

def Hurdles_Init():
        #Variables
        network = "hurdles_final"
        threshold = 0.8
        arguments = ['--model=networks/hurdles_final_2/ssd-mobilenet.onnx', '--labels=networks/hurdles_final_2/labels.txt', '--input-blob=input_0', '--output-cvg=scores', '--output-bbox=boxes']
        Overlay = "box,labels,conf"
 
         # load the object detection network
        net = jetson.inference.detectNet(network, arguments, threshold)

        return net

def Video_Source_init(type_of_camera):
        # create video sources & outputs
        input = jetson.utils.videoSource(type_of_camera)

        return input
        

def Imageprocessing(net, output_pic_name):
        #Variables
        input_URI = ""
        output_URI = ""
        Overlay = "box,labels,conf"

        # capture the next image
        glob.input.MUT.acquire()
        img = glob.input.value.Capture()
        glob.input.MUT.release()

        # #Send raw image to ROS topic
        # glob.pub.MUT.acquire()
        # glob.pub.image_raw.publish(msgify(Image, jetson.utils.cudaToNumpy(img), "rgb8"))
        # glob.pub.MUT.release()

        # detect objects in the image (with overlay)
        detections = net.Detect(img, overlay=Overlay)

        # print the detections
        #print("detected {:d} objects in image".format(len(detections)))

        #for detection in detections:
                #print("\nWe detected a " + net.GetClassDesc(detection.ClassID) + "\n")
                #print(type(detections[nbr]))

        # render the image
        #output.Render(img)
        #jetson.utils.saveImage(output_pic_name, img) #only save the image

        # update the title bar
        #output.SetStatus("{:s} | Network {:.0f} FPS".format(network, net.GetNetworkFPS()))

        # print out performance info
        #net.PrintProfilerTimes()

        return detections, img


if __name__ == "__main__":
        net_human = Multiped_Init()
        #print("\nMULTIPED INIT IS DONE \n")
        #time.sleep(3)

        net_hurdles = Hurdles_Init()
        #print("\nHurdles INIT IS DONE \n")
        #time.sleep(3)

        #We initialize the Camera as an input
        #print("\nCAMERA INIT IS DONE \n")
        glob.input.value = Video_Source_init("csi://0")
        #print("\nCAMERA INIT IS DONE \n")

        while True:
                print("\nSTART \n")
                   
                #print("\nSTART HUMAN DETECTION \n")
                #time.sleep(2)
                Imageprocessing(net_human, "./treated_current_pic_human_detection.png")

                #print("\nFINISH HUMAN DETECTION\n")
                #time.sleep(2)

                print("\nEND\n")

                time.sleep(10)
