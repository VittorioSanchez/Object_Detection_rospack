
##### authors: Raphael BENISTANT, Pedro GALVAO

# DETECTION SOFTWARE FOLDER

## File list:

### Custom packages:

* detectnet_camera_custom_siec_noCV2.py
* Detection_Thread.py
* ROS_publisher_Thread.py
* global_variables.py 
 

### main:
* main.py

## How it works:

The main software is in "main.py".

All the other functions are split in different scripts:

1. **global_variables.py**

Contains all the variables to be used simultaneaously in different python files.
The class *mutex_variable* defines a structure with variables used for publishing in all the ROS topics. The topics are the following:

* */detection*: for the class identifier of detected objects.
* *left*, *right*, *top*, *bottom*: those 4 topics have the coordinates delimiting the object's position.
* */detection_node/image*: for the images with bounding boxes drawn around each detected object.

This class also contains a *mutex* in order to avoid simultaneous access to shared variables from different threads.
In this file we create the global variable **pub** which type is *mutex_variable*. It is used in the *Detection_Thread* to send the object detection results (images + detections) to their respective topics.

Other global variables are used for measuring results in tests: The variables **tab_delta_time_loop_human**, **tab_delta_time_loop_hurdles** and **tab_delta_time_loop_ROS** are arrays where we register execution times for each loop for each thread. The variables **loop_counter_human**, **loop_counter_hurdles** and **loop_counter_ROS** count the number of loops executed in a test.

---

2. **detectnet_camera_custom_siec_noCV2.py**

This file has all the functions to initialize CNNs, the video input, and to process a single detection.

The functions *Multiped_Init* and *Hurdles_Init* are responsible for initializing respectively **multiped** and **ssd-mobilenet** with the right arguments (no need to specify them when calling those functions) including the threshold to send and to display the detection. For the human detection (**multiped**) we use a low threshold, of 50%, because we prioritize reducing the number of false negatives. For the other hurdles we use a threshold of 80%.

The function *Imageprocessing* is responsible for capturing an image from the video source and calling a neural network for processing. It returns the detection results and the image with bounding boxes.

---

3. **Detection_Thread.py**

Contains everything related to the real implementation of both the human CNN and the hurdles CNN. It carries out detections continuously and when it detects something, it sets a *detection flag* represented by the global variable **detection_flag** and sends a ROS message on the */detection* topic with the identifier of the detected object (see [classes corresponding numbers](./classes_training.txt).
Also, independently of having detected an object or not, the thread sends the video frame with boxes drawn on it indicating the positions of the detected objects on the */detection_node/image* ROS topic.

For the initialization, this class takes a single parameter *target*, which is a string indicating what neural network shall be used. This parameter may have the values *'human'* or *'hurdles'*.

The functions *human_detection_to_ROS_number* and *hurdles_detection_to_ROS_number* are responsible for converting the class identifiers from the outputs of each neural network to the corresponding identifiers to be used in the ROS topic. The classes and their corresponding class numbers for networks and ROS are in this [file](./classes_training.txt).

---

4. **ROS_publisher_Thread.py**

Contains the class ROS_publisher, which corresponds to a thread that continuously sends at 100Hz a ROS message to say nothing was detected (i.e. 0). It uses global variables for communicating with the detection threads. When **detection_flag** is set (i.e. something is detected), it clears the flag and stops sending ROS message for 2s. If nothing is detected during these 2s, it starts sending again.

---

5. **main.py**

In the main file the two previous thread classes are imported. First the ROS Topic is created, then 3 threads are declared and runned: the ROS publisher thread and 2 detection threads, one for humans and one for other hurdles.

## How to manually run the software:

Execute `python3.6 main.py` in a terminal opened in this folder.

## Further information:

Most of the software regarding the detection processing is an adaptation of the NVIDIA's github repository *_jetson_inference_* that you can find [here](https://github.com/dusty-nv/jetson-inference).
It contains the link to all API documentation as well as several tutorial for AI implementation and training.
