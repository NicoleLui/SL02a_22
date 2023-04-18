#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios


class Turtlebot3:
    def __init__(self):
        rospy.init_node('turtlebot3_controller')

        rospy.Subscriber('scan', LaserScan, self.laser_callback)

        self.vel_pub = rospy.Publisher('cmd_vel_rc100', Twist, queue_size=10)

        self.max_lin_vel = 0.2
        self.max_ang_vel = 2.0

        self.min_obstacle_dist = 0.4

        self.ctrl_msg = Twist()

        rospy.spin()


    def laser_callback(self, msg):
        if len(msg.ranges) == 0:
            return

        if min(msg.ranges) < self.min_obstacle_dist:
            self.ctrl_msg.linear.x = 0.0
            self.ctrl_msg.angular.z = 0.0
            self.vel_pub.publish(self.ctrl_msg)
        else:
            self.vel_pub.publish(self.ctrl_msg)


    def getKey(self):
        if os.name == 'nt':
            if sys.version_info[0] >= 3:
                return msvcrt.getch().decode()
            else:
                return msvcrt.getch()

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


    def control_loop(self):
        key = ''
        print("Control turtlebot3. Press w/a/s/d to move, q to quit.")
            
        while key != 'q':
            key = self.getKey()

            if key == 'w':
                print('received')
                self.ctrl_msg.linear.x = self.max_lin_vel
            elif key == 's':
                self.ctrl_msg.linear.x = -self.max_lin_vel
            elif key == 'a':
                self.ctrl_msg.angular.z = self.max_ang_vel
            elif key == 'd':
                self.ctrl_msg.angular.z = -self.max_ang_vel
            else:
                self.ctrl_msg.linear.x = 0.0
                self.ctrl_msg.angular.z = 0.0

            self.vel_pub.publish(self.ctrl_msg)




if __name__ == '__main__':
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    #rospy.init_node('turtlebot3_teleavoid')

    vel_pub = rospy.Publisher('cmd_vel_rc100', Twist, queue_size=10)
    tb3 = Turtlebot3()
    try:
        print('iam here try')
        tb3.control_loop()
    except: #rospy.ROSInterruptException:
        print('error')
    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        vel_pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)