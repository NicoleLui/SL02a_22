#!/usr/bin/env python

'''
Reference:
https://pyimagesearch.com/2021/10/06/opencv-contour-approximation/
'''

import rospy
import cv2
import argparse
import imutils
import numpy as np
from scipy.spatial import KDTree
from math import floor
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

class DrawingHandler:
    def __init__(self):
        self.ShowImages = True

        self.mobile_drawing = np.zeros((200,200,3),dtype=np.uint8)
        self.namespaces = ['tb101', 'tb104', 'tb105']
        self.tb3Locations = []
        self.goalCoors = []
        self.goalCoorsAssignedToRobots = []

        self.mobile_input_xy_pubs = []
        self.odom_subs = []
        
        rospy.init_node('drawing_handler', anonymous=True)
        self.mobile_drawing_sub = rospy.Subscriber("/mobile_drawing",CompressedImage,self.mobileDrawingCallback)
        for namespace in self.namespaces:

            # coordinate publisher
            topicName = '/' + namespace + '/mobile_input_xy'
            try:
                mobile_input_xy_pub = rospy.Publisher(topicName, Float32MultiArray,queue_size=1)
            except:
                mobile_input_xy_pub = None

            self.mobile_input_xy_pubs.append(mobile_input_xy_pub)
        
        self.rate = rospy.Rate(10)


    def mobileDrawingCallback(self,data):
        # update mobile_drawing
        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.mobile_drawing = image_np
        self.drawingToCoors()
        self.getRobotsLocation()
        self.assignCoorsToRobots()
        self.publishCoorsToRobots()
        # show the received image
        if self.ShowImages:
            cv2.imshow("Received drawing", image_np)
            cv2.imshow("Received drawing", image_np)
            cv2.waitKey(3)
        
    
    def drawingToCoors(self):
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

        # add the coor into self.goalCoors
        # map the coor in 3m X 3m field
        height, width, channels = image.shape
        for pt in approx:
            # x, y = pt[0][0], pt[0][1] #coor in pixels
            x, y = (float(pt[0][0])/width*2), (float(pt[0][1])/height*(-2))
            coors.append([x, y])
        # show the approximated contour image
        if self.ShowImages:
            cv2.imshow("Approximated Contour", output)
            cv2.imshow("Approximated Contour", output)
            cv2.waitKey(3)
        # write to self.goalCoors
        self.goalCoors = coors
        print("Drawing Handler: Generated goals: " , coors)


    def getRobotsLocation(self):
        namespaces = self.namespaces
        locations = []
        print("Drawing Handler: Reading robots' position...")

        for namespace in namespaces:
            # get odom for once
            topicName = '/' + namespace + '/odom'
            x, y = -1, -1
            try:
                msg = rospy.wait_for_message(topicName, Odometry)
                position = msg.pose.pose.position
                x, y = position.x, position.y
            except:
                pass
            locations.append([x, y])
            
        self.tb3Locations = locations


    def distanceBetween(self, pos1, pos2):
        return ( ((pos1[0]-pos2[0])**2) + ((pos1[1]-pos2[1])**2) )**0.5


    def distanceArrMultiRobot(self, robotPos, destPos): # [dis, dis, ...]
        distanceArrMultiRobot = []
        for thisRobot in robotPos:
            for thisDest in destPos:
                distanceArrMultiRobot.append(self.distanceBetween(thisRobot, thisDest))
        return distanceArrMultiRobot


    def assignCoorsToRobots(self):
        locations = self.tb3Locations
        goals = self.goalCoors
        goalAssignment = []

        distanceArrMultiRobot = self.distanceArrMultiRobot(locations, goals)

        for _ in range(len(locations)):
            goalAssignment.append([-999.0,-999.0])

        numRobots = len(locations)
        numGoals = len(goals)
        numAssignment = 0

        while ((numAssignment < numRobots) and (numAssignment < numGoals)):

            indexMin = np.argmin(distanceArrMultiRobot)
            indexRobot = int(floor(indexMin/numGoals))
            indexGoal = int(indexMin%numGoals)

            if goalAssignment[indexRobot] == [-999.0,-999.0]:
                # edit goal assignment
                goalAssignment[indexRobot] = goals[indexGoal]
                for i in range(numGoals):
                    distanceArrMultiRobot[(indexRobot*numGoals) + i] = 99999
                for i in range(numRobots):
                    distanceArrMultiRobot[(i*numGoals) + indexGoal] = 99999
                numAssignment += 1
            # else skip

        extraCount = 0
        for i in range(len(goalAssignment)):
            if goalAssignment[i] == [-999.0,-999.0]:
                x, y = 0 + extraCount*0.5, 0.0
                extraCount += 1
                goalAssignment[i] = [x, y]

        self.goalCoorsAssignedToRobots = goalAssignment


    def publishCoorsToRobots(self):
        namespaces = self.namespaces
        pubs = self.mobile_input_xy_pubs
        goalCoorsAssigned = self.goalCoorsAssignedToRobots

        for i in range(len(namespaces)):
            x, y = goalCoorsAssigned[i][0], goalCoorsAssigned[i][1]
            msg=Float32MultiArray()
            msg.data = [x,y]
            pubs[i].publish(msg)
            print('Drawing Handler: Destination published to ', namespaces[i], ' - ', str(msg.data))


    def publishCoorsToRobots_Random(self): # not considering distance, just distribut goals in order
        namespaces = self.namespaces
        goalCoors = self.goalCoors
        pubs = self.mobile_input_xy_pubs
        extraTb3 = 0
        for i in range(len(namespaces)):
            if i < len(goalCoors):
                x, y = goalCoors[i][0], goalCoors[i][1]
            else:
                x = 0 + extraTb3*0.5
                y = 0
                extraCount += 1
            msg=Float32MultiArray()
            msg.data = [x,y]
            pubs[i].publish(msg)
            print('Drawing Handler: published to ', namespaces[i], ': ', str(msg.data))


    def run(self):
        print('Drawing Handler: Program started. Waiting for input...')
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