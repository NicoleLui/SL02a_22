#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt
import math
import logging
import csv
import io
from datetime import datetime
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

# OBSTACLE DETECTION
STOP_DISTANCE = 0.25
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
LIDAR_SENSITIVITY = 1
avoidMode = ['STOP','TURN']
class TurtleBot:
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=False)


        rospy.on_shutdown(self.shutdown)

        self.obstacleMap = [0,0,0,0,0,0,0,0]
        self.oldeMap = [0,0,0,0,0,0,0,0]
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.odom = rospy.Subscriber('odom', Odometry, self.update_odometry)


        self.rate = rospy.Rate(10)


        self.position = None
        self.orientation = None


        while not rospy.is_shutdown():
            if self.position is not None and self.orientation is not None:
                break
            rospy.sleep(0.1)


        rospy.loginfo("TurtleBot3 Position: {}".format(self.position))
        rospy.loginfo("TurtleBot3 Orientation: {}".format(self.orientation))


    def update_odometry(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

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
        oldMap = [0,0,0,0,0,0,0,0]
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
                #print("ERROR XP")

            elif max(sorted(lidar_slice, reverse=False)[:LIDAR_SENSITIVITY]) <= SAFE_STOP_DISTANCE:
                obstacleMap[i]=1
                #print("Opps!Too Close!")
            else:
                obstacleMap[i]=0
            #print(min(lidar_slice))
        
        #print("Map:",obstacleMap)
        oldMap = self.obstacleMap
        self.obstacleMap = obstacleMap
        self.oldMap = oldMap


    # Provide new linear and angular vel accordig to obstacle map
    def obstacleAvoidVel(self, oldLinearVel, oldAngularVel, avoidMode = 'STOP'):

        obstacleMap = self.obstacleMap
        newLinearVel, newAngularVel = oldLinearVel, oldAngularVel
        oldMap=self.oldMap

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

        elif avoidMode == 'TURN':
            # condition any of obstacle sector 0,1,7 == 1 && burger moving forward
            if ((obstacleMap[0]==1 or obstacleMap[1]==1 or obstacleMap[7]==1 ) and oldLinearVel>0):
                newLinearVel=-0.15
                if oldAngularVel>0:
                    newAngularVel=-0.5
                elif oldAngularVel<0:
                    newAngularVel=0.5
                elif (obstacleMap[7]==1 or obstacleMap[6]==1): #sth on right,turn slightly left
                    newAngularVel=-1
                else : newAngularVel=1
                
                cmd = Twist()
                cmd.linear.x ,cmd.angular.z = newLinearVel,newAngularVel
                self.cmd_vel.publish(cmd)
                rospy.sleep(0.1)
            # condition any of obstacle sector 3,4,5 == 1 && burger moving backward
            elif ((obstacleMap[3]==1 or obstacleMap[4]==1 or obstacleMap[5]==1) and oldLinearVel<0):
                newLinearVel=0.15
                if oldAngularVel>0:
                    newAngularVel=-0.5
                elif oldAngularVel<0:
                    newAngularVel=0.5
                elif (obstacleMap[3]==1 or obstacleMap[2]==1): #sth on left,turn slightly right
                    newAngularVel=1
                else : newAngularVel=-1
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
            # condition any of obstacle sector 1,2 == 1 && burger turn left && front
            elif (((obstacleMap[1]==1 and oldMap[1]!=obstacleMap[1]) or (obstacleMap[2]==1 and oldMap[2]!=obstacleMap[2])) and oldAngularVel>0 and oldLinearVel>0):
                newAngularVel=-oldAngularVel
                newLinearVel=-oldLinearVel
            # condition any of obstacle sector 5,6 == 1 && burger turn right and back
            elif (((obstacleMap[5]==1 and oldMap[5]!=obstacleMap[5]) or (obstacleMap[6]==1 and oldMap[6]!=obstacleMap[6])) and oldAngularVel<0 and oldLinearVel<0):
                newAngularVel=-oldAngularVel
                newLinearVel=-oldLinearVel
            # condition any of obstacle sector 1,2 == 1 && burger turn left && back
            elif (((obstacleMap[2]==1 and oldMap[2]!=obstacleMap[2]) or (obstacleMap[3]==1 and oldMap[3]!=obstacleMap[3])) and oldAngularVel<0 and  oldLinearVel<0):
                newAngularVel=-oldAngularVel
                newLinearVel=-oldLinearVel
            # condition any of obstacle sector 6,7 == 1 && burger turn right and front
            elif (((obstacleMap[6]==1 and oldMap[6]!=obstacleMap[6]) or (obstacleMap[7]==1 and oldMap[7]!=obstacleMap[7])) and oldAngularVel>0 and oldLinearVel>0):
                newAngularVel=-oldAngularVel
                newLinearVel=-oldLinearVel
            '''
            

        return newLinearVel, newAngularVel

    def move_to(self, x, y, tolerance=0.01, linear_speed=0.05, angular_speed=0.5):


        while not rospy.is_shutdown():
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
            print('andgle:%f\torientation%f\tdtheta%f\t\tdistance%f')% (angle, theta,dtheta, distance)
            #msg = str(str(angle)+','+str(theta)+','+str(dtheta)+','+str(distance))
            #logging.debug(msg)
            
            if abs(dtheta) > 0.5:
                cmd = Twist()
                if dtheta > 0:
                    cmd.angular.z = angular_speed
                else:
                    cmd.angular.z = -angular_speed
                #self.cmd_vel.publish(cmd)
                print("now >1")
            elif abs(dtheta) > 0.2:
                cmd = Twist()
                if dtheta > 0:
                    cmd.angular.z = angular_speed/2
                else:
                    cmd.angular.z = -angular_speed/2
                #self.cmd_vel.publish(cmd)
                print("now>0.2")
            else:
                cmd = Twist()
                cmd.linear.x = linear_speed
                cmd.angular.z =0.0
                #self.cmd_vel.publish(cmd)
                print("Lets GOOOOO!")

            #add obstacle avoidance in it
            self.getObstacleMap()
            cmd.linear.x ,cmd.angular.z = self.obstacleAvoidVel(cmd.linear.x,cmd.angular.z,'TURN')
            #publish
            self.cmd_vel.publish(cmd)
            #run a while
            self.rate.sleep()
        rospy.loginfo("Reached target position ({}, {})".format(x, y))
        rospy.loginfo("Actual Position ({},{})".format(self.position.x,self.position.y))


    def run(self):
                                            
        while not rospy.is_shutdown():
            #receive app command input
            #Input_x=rospy.Subscriber('goto',)
            #Input_y=rospy.Subscriber('goto',)
            x = float(input("Enter the x coordinate (or q to quit): "))
            if x == 'q':
                break
            y = float(input("Enter the y coordinate: "))


            self.move_to(x, y)
            


if __name__ == '__main__':

    #logging.basicConfig(filename=('log_moveToCoor'+str(datetime.now())+'.txt'), encoding='utf-8' , level=logging.DEBUG)

    try:
        tb = TurtleBot()
        tb.run()


    except rospy.ROSInterruptException:
        pass