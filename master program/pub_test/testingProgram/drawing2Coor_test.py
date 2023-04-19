#!/usr/bin/env python

import rospy
import numpy as np
import cv2

import argparse
import imutils
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray

img=''
img = cv2.imread('a.png')
print(img)
rospy.init_node('drawing_handler', anonymous=True)
pub = rospy.Publisher("/mobile_drawing",CompressedImage,queue_size=1)

'''count = 0
while not rospy.is_shutdown():
    #Create CompressedIamge
    msg = CompressedImage()
    msg.format = "png"
    msg.data = np.array(cv2.imencode('.png', img)[1]).tostring()
    # Publish new image
    pub.publish(msg)
    print('published.')
    rospy.sleep(3)'''
#Create CompressedIamge
msg = CompressedImage()
msg.format = "png"
msg.data = np.array(cv2.imencode('.png', img)[1]).tostring()
# Publish new image
pub.publish(msg)
print('published.')
rospy.sleep(3)
pub.publish(msg)
print('published.')
'''
    msg = CompressedImage()
    msg.format = "jpeg"
    msg.data = img
    pub.publish(msg)
    print('published.')'''
