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
    def _init_(self):
        rospy.init_node('turtlebot3_controller')

        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

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

    def control_loop(self):
        key = ''

        while key != 'q':
            print("Control turtlebot3. Press w/a/s/d to move, q to quit.")
            key = getKey()
            
            if key == 'w':
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

def getKey():
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

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)



if __name__ == '__main__':

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    try:
        tb3 = Turtlebot3()
        tb3.control_loop()
    except rospy.ROSInterruptException:
        pass


    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
