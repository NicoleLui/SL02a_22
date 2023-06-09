﻿#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, sin, cos
import math
import logging
import csv
import io
import time
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import string

# OBSTACLE DETECTION
STOP_DISTANCE = 0.5
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
LIDAR_SENSITIVITY = 1
WAIT_TIME = 60
avoidMode = ['STOP','NAIVE']


class TurtleBot:
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=False)


        rospy.on_shutdown(self.shutdown)
        self.ObstaclePointList=[]
        self.obstacleMap = [0,0,0,0,0,0,0,0]
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.odom = rospy.Subscriber('odom', Odometry, self.update_odometry)
        self.ObstaclePointPub=rospy.Publisher('/obstaclePoint',Float32MultiArray, queue_size = 10)
        rospy.Subscriber('/obstaclePoint', Float32MultiArray, self.ObstaclePointUpdate)
        self.rate = rospy.Rate(10)
        self.xy_input = rospy.Subscriber('mobile_input',string,self.updateXY_input)

        self.position = None
        self.orientation = None
        self.StartTime = 0
        self.frontObstacleDistance=-1
        self.inputX='o'
        self.inputY='o'

        while not rospy.is_shutdown():
            if self.position is not None and self.orientation is not None:
                break
            rospy.sleep(0.1)


        rospy.loginfo("TurtleBot3 Position: {}".format(self.position))
        rospy.loginfo("TurtleBot3 Orientation: {}".format(self.orientation))


    def update_odometry(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation

    def updateXY_input(self):
        self.inputX=self.xy_input.data[0]
        self.inputY=self.xy_input.data[1]

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


    def ObstaclePointUpdate(self,data):
        x = data.data[0]
        y = data.data[1]
        self.ObstaclePointList.append(x)
        self.ObstaclePointList.append(y)
        #receive obstacle point and save!



    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
        scan_filter = scan.ranges

       
        samples = len(scan.ranges)  # The number of samples is defined in 
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 1            # 1 <= samples_view <= samples
        
        '''
        if samples_view > samples:
            samples_view = samples

        if samples_view is 1:
            scan_filter.append(scan.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2
            
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)
        '''
        
        for i in range(samples):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0

        return scan_filter

    # Get obstacle map arr
    def getObstacleMap(self):
        obstacleMap = [0,0,0,0,0,0,0,0]
        lidar_distances = self.get_scan()
        # loop for 8 direction
        for i in range(8):
            # slice lidar reading into 8 parts
            if i == 0:
                lidar_slice = lidar_distances[-23:] + lidar_distances[0:22]
            else:
                lidar_slice=lidar_distances[(-23+45*i):(0+45*i)] + lidar_distances[(0+45*i):(22+45*i)]
            
            lidar_slice=(list(filter(lambda num: num != 0.0, lidar_slice)))   
            if len(lidar_slice)==0:
                #print("ALL zeros in section ",i)
                obstacleMap[i]=-1

            elif max(sorted(lidar_slice, reverse=False)[:LIDAR_SENSITIVITY]) <= SAFE_STOP_DISTANCE:
                obstacleMap[i]=1
                if i==0 and obstacleMap[0] == 1:
                    self.frontObstacleDistance=min(lidar_slice)
                #print("Opps!Too Close!")
            else:
                obstacleMap[i]=0
            #print(min(lidar_slice))
        
        #print("Map:",obstacleMap)
        self.obstacleMap = obstacleMap


    # Provide new linear and angular vel accordig to obstacle map
    def obstacleAvoidVel(self, oldLinearVel, oldAngularVel, avoidMode = 'STOP'):

        obstacleMap = self.obstacleMap
        newLinearVel, newAngularVel = oldLinearVel, oldAngularVel

        if avoidMode == 'STOP':
            # condition any of obstacle sector 0,1,7 == 1 && burger moving forward
            if ((obstacleMap[0]==1 or obstacleMap[1]==1 or obstacleMap[7]==1) and oldLinearVel>0):
                newLinearVel=0.0
                newAngularVel=0.0
            # condition any of obstacle sector 3,4,5 == 1 && burger moving backward
            elif ((obstacleMap[3]==1 or obstacleMap[4]==1 or obstacleMap[5]==1) and oldLinearVel<0):
                newLinearVel=0.0
                newAngularVel=0.0
            # condition any of obstacle sector 1,2 == 1 && burger turn left && front
            elif ((obstacleMap[1]==1 or obstacleMap[2]==1) and oldAngularVel>0 and oldLinearVel>0):
                newAngularVel=0.0
            # condition any of obstacle sector 5,6 == 1 && burger turn right and back
            elif ((obstacleMap[5]==1 or obstacleMap[6]==1) and oldAngularVel<0 and oldLinearVel<0):
                newAngularVel=0.0
            # condition any of obstacle sector 1,2 == 1 && burger turn left && back
            elif ((obstacleMap[2]==1 or obstacleMap[3]==1) and oldAngularVel<0 and  oldLinearVel<0):
                newAngularVel=0.0
            # condition any of obstacle sector 6,7 == 1 && burger turn right and front
            elif ((obstacleMap[6]==1 or obstacleMap[7]==1) and oldAngularVel>0 and oldLinearVel>0):
                newAngularVel=0.0

        elif avoidMode == 'NAIVE':
            # condition any of obstacle sector 0,1,7 == 1 && burger moving forward
            if ((obstacleMap[0]==1 or obstacleMap[1]==1 or obstacleMap[7]==1 )):
                print("Something Detected, turning...")
                newLinearVel=0.08
                newAngularVel=-0.6
                if (obstacleMap[7]==1):
                    newAngularVel=0.6

            else:
                newLinearVel=oldLinearVel
                newAngularVel=oldAngularVel
                '''
                cmd = Twist()
                cmd.linear.x = 0
                cmd.angular.z = newAngularVel
                self.cmd_vel.publish(cmd)
                rospy.sleep(2)
                cmd.linear.x = newLinearVel
                cmd.angular.z = 0
                self.cmd_vel.publish(cmd)
                rospy.sleep(2)
                '''
            '''    
            # condition any of obstacle sector 1,2 == 1 && burger turn left && front
            elif (((obstacleMap[1]==1) or (obstacleMap[2]==1 )) and oldAngularVel>0 and oldLinearVel>0):
                newAngularVel=-oldAngularVel
                newLinearVel=-oldLinearVel
            # condition any of obstacle sector 5,6 == 1 && burger turn right and back
            elif ((obstacleMap[5]==1) or (obstacleMap[6]==1) and oldAngularVel<0 and oldLinearVel<0):
                newAngularVel=-oldAngularVel
                newLinearVel=-oldLinearVel
            # condition any of obstacle sector 1,2 == 1 && burger turn left && back
            elif ((obstacleMap[2]==1 or obstacleMap[3]==1) and oldAngularVel<0 and  oldLinearVel<0):
                newAngularVel=-oldAngularVel
                newLinearVel=-oldLinearVel
            # condition any of obstacle sector 6,7 == 1 && burger turn right and front
            elif ((obstacleMap[6]==1 or obstacleMap[7]==1) and oldAngularVel>0 and oldLinearVel>0):
                newAngularVel=-oldAngularVel
                newLinearVel=-oldLinearVel    
            '''

        return newLinearVel, newAngularVel

    def move_to(self, x, y, tolerance=0.01, linear_speed=0.05, angular_speed=0.5):
        self.StartTime=time.time()

        while not rospy.is_shutdown():
            #after a certain time limit still not arrive
            if time.time()-self.StartTime > WAIT_TIME:
                print("Opps...There might be something on that spot.")
                print("Will Stay here wait for the next instruction.")
                msg=Float32MultiArray()
                msg.data = [x,y]
                self.ObstaclePointPub.publish(msg)
                cmd = Twist()
                cmd.angular.z =0.0
                cmd.linear.x = 0.0
                self.cmd_vel.publish(cmd)
                #publish obstacle point,stop
                break

            #check if it is in on the obstacle point list
            for i in range (0,len(self.ObstaclePointList),2):
                if x+SAFE_STOP_DISTANCE >= self.ObstaclePointList[i] and x-SAFE_STOP_DISTANCE <= self.ObstaclePointList[i] and y+SAFE_STOP_DISTANCE >= self.ObstaclePointList[i+1] and y-SAFE_STOP_DISTANCE <= self.ObstaclePointList[i+1]:
                    print("This point have some obstacles, please pick another point!")
                    cmd = Twist()
                    cmd.angular.z =0.0
                    cmd.linear.x = 0.0
                    self.cmd_vel.publish(cmd)
                    #publish stop
                    return

            dx = x - self.position.x
            dy = y - self.position.y
            distance = sqrt(dx**2 + dy**2)
            if distance>=0.3:
                    linear_speed=0.2
            elif distance>=0.1:
                    linear_speed=0.1
            elif distance>=0.05:
                    linear_speed=0.05
            else:
                linear_speed=0.01

            if distance < tolerance:
                print("Arrived")
                cmd = Twist()
                cmd.angular.z =0.0
                cmd.linear.x = 0.0
                self.cmd_vel.publish(cmd)
                break
            (roll, pitch, theta) = euler_from_quaternion([self.orientation.x, self.orientation.y, self.orientation.z,self.orientation.w])

            angle = atan2(dy, dx)
            dtheta = angle - theta
            #print('andgle:%f\torientation%f\tdtheta%f\t\tdistance%f')% (angle, theta,dtheta, distance)
            #msg = str(str(angle)+','+str(theta)+','+str(dtheta)+','+str(distance))
            
            if abs(dtheta) > 0.5:
                cmd = Twist()
                if dtheta > 0:
                    cmd.angular.z = angular_speed
                else:
                    cmd.angular.z = -angular_speed

            elif abs(dtheta) > 0.2:
                cmd = Twist()
                if dtheta > 0:
                    cmd.angular.z = angular_speed/2
                else:
                    cmd.angular.z = -angular_speed/2
            else:
                cmd = Twist()
                cmd.linear.x = linear_speed
                cmd.angular.z =0.0

            #add obstacle avoidance in it
            self.getObstacleMap()
            cmd.linear.x ,cmd.angular.z = self.obstacleAvoidVel(cmd.linear.x,cmd.angular.z,'NAIVE')
            if self.obstacleMap[0]==1:
                obsX=self.frontObstacleDistance*sin(theta)+self.position.x
                obsY=self.frontObstacleDistance*cos(theta)+self.position.y
                inList=False
                for i in range (0,len(self.ObstaclePointList),2):
                    if obsX+SAFE_STOP_DISTANCE >= self.ObstaclePointList[i] and obsX-SAFE_STOP_DISTANCE <= self.ObstaclePointList[i] and obsY+SAFE_STOP_DISTANCE >= self.ObstaclePointList[i+1] and obsY-SAFE_STOP_DISTANCE <= self.ObstaclePointList[i+1]:
                        inList=True
                        break
                if not(inList):
                    msg=Float32MultiArray()
                    msg.data = [obsX,obsY]
                    self.ObstaclePointPub.publish(msg)

            #publish
            self.cmd_vel.publish(cmd)
            #run a while
            self.rate.sleep()
        rospy.loginfo("Reached target position ({}, {})".format(x, y))
        rospy.loginfo("Actual Position ({},{})".format(self.position.x,self.position.y))
        self.xy_input='oo'


    def run(self):
                                            
        while not rospy.is_shutdown():
            #receive app command input
            x=self.inputX
            y=self.inputY
            if x=='o':
                continue
            #wait until subscriber receive coordinate
            #x = float(input("Enter the x coordinate (or q to quit): "))
            if x == 'q':
                break
            #y = float(input("Enter the y coordinate: "))
            print("ObstaclePoints: ",self.ObstaclePointList)
            self.move_to(x, y)
            


if __name__ == '__main__':

    #logging.basicConfig(filename=('log_moveToCoor'+str(datetime.now())+'.txt'), encoding='utf-8' , level=logging.DEBUG)

    try:
        tb = TurtleBot()
        tb.run()


    except rospy.ROSInterruptException:
        pass
