#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import math
from sensor_msgs.msg import LaserScan

rospy.init_node('turtlebot3_controller')

rospy.Subscriber('scan', LaserScan)
samples_view = 1
while(1):

    
    scan = rospy.wait_for_message('scan', LaserScan)
    scan_filter = []

    samples = len(scan.ranges)  # The number of samples is defined in 
                                # turtlebot3_<model>.gazebo.xacro file,
                                # the default is 360.
            # 1 <= samples_view <= samples

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

    for i in range(samples_view):
        if scan_filter[i] == float('Inf'):
            scan_filter[i] = 3.5
        elif math.isnan(scan_filter[i]):
            scan_filter[i] = 0
        print(i,  ': ',  scan_filter[i]) 

    samples_view+=1
    print(samples_view)