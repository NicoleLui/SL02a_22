#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class Turtlebot3:
    def __init__(self):
        # 初始化ROS節點
        rospy.init_node('turtlebot3_controller')


        # 訂閱激光雷達訊息
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)


        # 發佈機器人控制訊息
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


        # 設置機器人最大線速度和角速度
        self.max_lin_vel = 0.2
        self.max_ang_vel = 2.0


        # 設置障礙物最小距離
        self.min_obstacle_dist = 0.4


        # 初始化控制訊息
        self.ctrl_msg = Twist()


        # 開始ROS循環
        rospy.spin()


    def laser_callback(self, msg):
        # 檢查激光雷達數據
        if len(msg.ranges) == 0:
            return


        # 檢查前方是否有障礙物
        if min(msg.ranges) < self.min_obstacle_dist:
            # 停止機器人
            self.ctrl_msg.linear.x = 0.0
            self.ctrl_msg.angular.z = 0.0
            self.vel_pub.publish(self.ctrl_msg)
        else:
            # 繼續機器人運動
            self.vel_pub.publish(self.ctrl_msg)


    def control_loop(self):
        # 初始化方向鍵
        key = ''


        # 開始手動控制
        while key != 'q':
            # 接收方向鍵輸入
            key = raw_input("Control turtlebot3. Press w/a/s/d to move, q to quit.")


            # 根據方向鍵更新機器人控制訊息
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


            # 發佈機器人控制訊息
            self.vel_pub.publish(self.ctrl_msg)


if __name__ == '__main__':
    try:
        tb3 = Turtlebot3()
        tb3.control_loop()
    except rospy.ROSInterruptException:
        pass