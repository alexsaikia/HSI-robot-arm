#!/usr/bin/python3 
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from ximea import xiapi
from cv_bridge import CvBridge, CvBridgeError

# This node is used to capture and save video from a XIMEA camera
# The video is saved in /media/alex/Data/data
# The video is saved in .avi format
# The video is saved with the name "video" + the number of the video
# The video is saved with a frame rate of 30 fps
# The video is saved with the native max resolution of the camera

# The node is launched with the launch file "dual_capture.launch"
# The launch file is located in HSI-robot-arm/src/acquisition/launch/dual_capture.launch
# The launch file can be launched with the command "roslaunch dual_capture.launch"

# The node is stopped with the command "Ctrl + C"

# The node publishes the video in the topic "dual_video"
# The topic is of type sensor_msgs/Image
# The topic is published with a rate of 30 fps

# Create the node
rospy.init_node('dual_capture', anonymous=True)

# Create the publisher
pub = rospy.Publisher('dual_video', Image, queue_size=10)

# Create the bridge
bridge = CvBridge()

# Create the video writer
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('/media/alex/Data/data/video1.avi', fourcc, 30.0, (1920, 1200))

# Create the camera
cam = xiapi.Camera()
cam.open_device()
cam.set_exposure(10000)
cam.set_param('width', 1920)
cam.set_param('height', 1200)
cam.set_param('downsampling_type', 'XI_SKIPPING')
cam.set_param('downsampling', 2)
cam.set_param('framerate', 30)
cam.set_param('imgdataformat', 'XI_RGB24')
cam.set_acq_timing_mode('XI_ACQ_TIMING_MODE_FREE_RUN')
cam.start_acquisition()

# Create the loop
while not rospy.is_shutdown():
    # Get the image from the camera
    img = cam.get_image()
    
    # Convert the image to a numpy array
    img = img.get_image_data_numpy()
    
    # Convert the image to a ROS image
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = bridge.cv2_to_imgmsg(img, "rgb8")
    
    # Publish the image
    pub.publish(img)
    
    # Save the image
    img = bridge.imgmsg_to_cv2(img, "rgb8")
    out.write(img)

       
        