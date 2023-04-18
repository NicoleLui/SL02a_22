#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt
import logging
import csv
import io
from datetime import datetime
from tf.transformations import euler_from_quaternion
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

class TurtleBot:
    def __init__(self):
        rospy.init_node('turtlebot_controller')

        rospy.on_shutdown(self.shutdown)

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
            msg = str(str(angle)+','+str(theta)+','+str(dtheta)+','+str(distance))
            logging.debug(msg)
            
            if abs(dtheta) > 0.5:
                cmd = Twist()
                if dtheta > 0:
                    cmd.angular.z = angular_speed
                else:
                    cmd.angular.z = -angular_speed
                self.cmd_vel.publish(cmd)
                print("now >1")
            elif abs(dtheta) > 0.2:
                cmd = Twist()
                if dtheta > 0:
                    cmd.angular.z = angular_speed/2
                else:
                    cmd.angular.z = -angular_speed/2
                self.cmd_vel.publish(cmd)
                print("now>0.2")
            else:
                cmd = Twist()
                cmd.linear.x = linear_speed
                cmd.angular.z =0.0
                self.cmd_vel.publish(cmd)
                print("Lets GOOOOO!")

            self.rate.sleep()


        rospy.loginfo("Reached target position ({}, {})".format(x, y))
        rospy.loginfo("Actual Position ({},{})".format(self.position.x,self.position.y))


    def run(self):
                                            
        while not rospy.is_shutdown():
            x = float(input("Enter the x coordinate (or q to quit): "))
            if x == 'q':
                break
            y = float(input("Enter the y coordinate: "))


            self.move_to(x, y)
            


if __name__ == '__main__':
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    logging.basicConfig(filename=('log_moveToCoor'+str(datetime.now())+'.txt'), encoding='utf-8' , level=logging.DEBUG)

    try:
        tb = TurtleBot()
        tb.run()


    except rospy.ROSInterruptException:
        pass

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)