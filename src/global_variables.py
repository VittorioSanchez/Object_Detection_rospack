from threading import Lock, Event

#ROS library
import rospy
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

class mutex_variable:
    def __init__(self):
        self.value = rospy.Publisher('detection', UInt8, queue_size=10)
        #self.image_raw = rospy.Publisher('/detection_node/image_raw', Image, queue_size=10)
        self.image = rospy.Publisher('/detection_node/image', Image, queue_size=10)
        self.left = rospy.Publisher('left', Float64, queue_size=10)
        self.right = rospy.Publisher('right', Float64, queue_size=10)
        self.top = rospy.Publisher('top', Float64, queue_size=10)
        self.bottom = rospy.Publisher('bottom', Float64, queue_size=10)
        self.MUT = Lock()



detection_flag = mutex_variable()
detection_flag.value = 0

pub = mutex_variable()
input = mutex_variable()

## Only used to measure time execution
# tab_delta_time_loop_human = []
# tab_delta_time_loop_hurdles = []
# tab_delta_time_loop_ROS = []

# NUMBER_OF_LOOPS_FOR_TEST = 100

# loop_counter_human = 0
# loop_counter_hurdles = 0
# loop_counter_ROS = 0
