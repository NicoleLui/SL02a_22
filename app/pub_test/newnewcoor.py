#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


# 定義全局變量 move_base，用於發佈 MoveBaseGoal
move_base = None


# 初始化 move_base
def init_move_base():
    global move_base
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("等待 move_base 服務啟動...")
    move_base.wait_for_server(rospy.Duration(60))
    rospy.loginfo("move_base 服務已啟動！")


# 定義 move_base_goal 函數
def move_base_goal(x, y, theta=0):
    # 創建一個 MoveBaseGoal 類型的對象
    goal = MoveBaseGoal()


    # 設置目標位置
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position = Point(x, y, 0)
    q = quaternion_from_euler(0, 0, theta)
    goal.target_pose.pose.orientation = Quaternion(*q)


    # 發佈目標位置，讓機器人開始移動
    move_base.send_goal(goal)


    # 等待機器人到達目標位置
    move_base.wait_for_result()


    # 獲取機器人的狀態
    state = move_base.get_state()


    # 返回機器人的狀態
    return state


if __name__ == '__main__':
    # 初始化 ROS 节点
    rospy.init_node("move_to_coordinate")


    # 初始化 move_base
    init_move_base()


    # 讓機器人移動到指定的坐標
    while not rospy.is_shutdown():
        x = float(input("請輸入 x 坐標: "))
        y = float(input("請輸入 y 坐標: "))
        theta = float(input("請輸入機器人的朝向（角度）: "))
        theta = theta / 180.0 * 3.14159 # 將角度轉換為弧度
        result = move_base_goal(x, y, theta)
        if result == GoalStatus.SUCCEEDED:
            rospy.loginfo("已到達目標位置！")
        else:
            rospy.loginfo("無法到達目標位置！")