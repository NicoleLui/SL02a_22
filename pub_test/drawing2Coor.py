#!/usr/bin/env python

'''
Reference:
https://pyimagesearch.com/2021/10/06/opencv-contour-approximation/
'''

import rospy
import numpy as np
import cv2
import argparse
import imutils
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray

class DrawingHandler:
    def __init__(self):
        self.namespaces = ['tb101', 'tb104', 'tb105']
        self.mobile_drawing = np.zeros((200,200,3),dtype=np.uint8)
        self.coors = []
        self.mobile_input_xy_pubs = []
        
        rospy.init_node('drawing_handler', anonymous=True)
        self.mobile_drawing_sub = rospy.Subscriber("/mobile_drawing",CompressedImage,self.mobile_drawing_callback)
        for namespace in self.namespaces:
            topicName = '/' + namespace + '/mobile_input_xy'
            pub = rospy.Publisher(topicName, Float32MultiArray,queue_size=1)
            self.mobile_input_xy_pubs.append(pub)
        
        self.rate = rospy.Rate(10)

    def mobile_drawing_callback(self,data):
        # update mobile_drawing
        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.mobile_drawing = image_np
        self.drawing2coors()
        self.publish_coor_to_robot()
        # show the received image
        cv2.imshow("Received drawing", image_np)
        cv2.imshow("Received drawing", image_np)
        cv2.waitKey(3)
        

    def drawing2coors(self):
        image = self.mobile_drawing
        coors = []
        
        # construct the argument parser and parse the arguments
        ap = argparse.ArgumentParser()
        ap.add_argument("-i", "--image", type=str, default=image,
			help="path to input image")
        args = vars(ap.parse_args())

        # convert the image to grayscale and threshold it
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 200, 255,
			cv2.THRESH_BINARY_INV)[1]

		# find the largest contour in the threshold image
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        c = max(cnts, key=cv2.contourArea)

        # draw the shape of the contour on the output image, compute the
		# bounding box, and display the number of points in the contour
        output = image.copy()
        cv2.drawContours(output, [c], -1, (0, 255, 0), 3)
        (x, y, w, h) = cv2.boundingRect(c)
        text = "original, num_pts={}".format(len(c))
        cv2.putText(output, text, (x, y - 15), cv2.FONT_HERSHEY_SIMPLEX,
			0.9, (0, 255, 0), 2)
        print("[INFO] {}".format(text))
        #cv2.imshow("Original Contour", output)

		# to demonstrate the impact of contour approximation, let's loop
		# over a number of epsilon sizes
        for eps in np.linspace(0.001, 0.05, 10):
            # approximate the contour
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, eps * peri, True)
            # print progress
            text = "eps={:.4f}, num_pts={}".format(eps, len(approx))
            print("[INFO] {}".format(text))

        # draw the approximated contour on the image
        output = image.copy()
        cv2.drawContours(output, [approx], -1, (0, 255, 0), 3)
        # write the shape on the image
        if len(approx) == 3: shape = 'Triangle'
        elif len(approx) == 4: shape = 'Quadrilateral'
        elif len(approx) == 5: shape = 'Pentagon'
        elif len(approx) == 6: shape = 'Hexagon'
        else: shape = 'Circle'
        text = "eps={:.4f}, num_pts={}".format(eps, len(approx))
        cv2.putText(output, shape, (x, y - 15), cv2.FONT_HERSHEY_SIMPLEX,
		0.9, (0, 255, 0), 2)

        # add the coor into self.coors
        # map the coor in 3m X 3m field
        height, width, channels = image.shape
        for pt in approx:
            # x, y = pt[0][0], pt[0][1] #coor in pixels
            x, y = (float(pt[0][0])/width*3), (float(pt[0][1])/height*3)
            coors.append([x, y])
		# show the approximated contour image
        cv2.imshow("Approximated Contour", output)
        cv2.imshow("Approximated Contour", output)
        cv2.waitKey(3)
        # write to self.coors
        self.coors = coors
        print(coors)
        

    def publish_coor_to_robot(self):
        namespaces = self.namespaces
        coors = self.coors
        pubs = self.mobile_input_xy_pubs
        extraTb3 = 0
        for i in range(len(namespaces)):
            if i < len(coors):
                x, y = coors[i][0], coors[i][1]
            else:
                x = 0 + extraTb3*0.5
                y = 0
                extraCount += 1
            msg=Float32MultiArray()
            msg.data = [x,y]
            pubs[i].publish(msg)
            print('published to ' + namespaces[i] + ': ' + str(msg.data))


    def run(self):
        print('drawing to coordinates start running...')
        while not rospy.is_shutdown():
            # keep the programme awake
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # Once there is received msg, the handler will react and publish result
        dh = DrawingHandler()
        dh.run()
        
    except rospy.ROSInterruptException:
        pass