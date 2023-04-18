#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from math import atan2, sqrt, cos, sin, tan
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# OBSTACLE DETECTION CONSTANTS
ALERT_DISTANCE = 0.5
STOP_DISTANCE = 0.25
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
LIDAR_SENSITIVITY = 1
TOLERANCE = 0.01
BURGER_MAX_ANGULAR_VEL = 1.2
BURGER_MAX_LINEAR_VEL = 0.22


def mapRadianNegPiToPosPi(radian):
    n = int(radian/math.pi)
    if n<0:
        if n%2 == 1:
            n = (-n)+1
        elif n%2 == 0:
            n = (-n)
    elif n>0:
        if n%2 == 1:
            n = (-n)-1
        elif n%2 == 0:
            n = (-n)
    return radian + (n * math.pi)


class TurtleBot:
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.goal = [0, 0]
        self.pos = [0, 0, 0]
        self.vel = [0, 0, 0]
        self.angular_vel = 0
        self.linear_vel = 0
        self.obstacle_map = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.odom = rospy.Subscriber('odom', Odometry, self.update_odometry)
        self.scan_ranges = rospy.Subscriber('scan', LaserScan, self.update_scan_ranges_and_map)
        self.initial_pose = rospy.Publisher('initialpose',PoseWithCovarianceStamped, queue_size=1)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.amcl_pose = rospy.Publisher('amcl_pose', PoseWithCovarianceStamped, queue_size=1)

        self.rate = rospy.Rate(10)

        self.goal_reached = True

        rospy.loginfo("TurtleBot3 Position: {}".format(self.pos[0], self.pos[1]))
        rospy.loginfo("TurtleBot3 Orientation: {}".format(self.pos[2]))

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


    def update_odom_position(self, x=0.0, y=0.0):
        odom = Odometry()
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        print("odom position update")
        self.odom_pub.publish(odom)


    def update_amcl_pose(self, x=0.0, y=0.0, angle=0):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = 0.0

        yaw = math.radians(angle)
        quat = quaternion_from_euler(0,0,yaw)
        pose.pose.pose.orientation.x = quat[0]
        pose.pose.pose.orientation.y = quat[1]
        pose.pose.pose.orientation.z = quat[2]
        pose.pose.pose.orientation.w = quat[3]

        print("initial amcl pose:x: ", x, " y: ", y, "yaw = ", yaw)
        self.amcl_pose.publish(pose)


    def update_initial_pose(self, x=0.0, y=0.0, angle=0):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = 0.0

        yaw = math.radians(angle)
        quat = quaternion_from_euler(0,0,yaw)
        pose.pose.pose.orientation.x = quat[0]
        pose.pose.pose.orientation.y = quat[1]
        pose.pose.pose.orientation.z = quat[2]
        pose.pose.pose.orientation.w = quat[3]

        print("x: ", x, " y: ", y, "yaw = ", yaw)
        self.initial_pose.publish(pose)


    def update_odometry(self, odom_data):
        x = odom_data.pose.pose.position.x
        y = odom_data.pose.pose.position.y
        orientation = odom_data.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        self.pos = [x, y, yaw]
        self.vel = [odom_data.twist.twist.linear.x, odom_data.twist.twist.linear.y, odom_data.twist.twist.angular.z]
        self.orientation = [roll,pitch,yaw]
        #print("ODOM:  ",x,y)


    def update_scan_ranges_and_map(self, scanData):
        self.scan_ranges = scanData.ranges
        self.update_obstacle_map()
    

    # Update obstacle map arr
    def update_obstacle_map(self):
        obstacleMap = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        ranges = self.scan_ranges
        
        for i in range(8):
            current_direction_ranges = []
            ### FRONT         obstacleMap[0]
            if i == 0: current_direction_ranges = [ranges[-20], ranges[-10], ranges[0], ranges[10], ranges[20]]
            ### FRONT-LEFT    obstacleMap[1]
            elif i == 1: current_direction_ranges = [ranges[30], ranges[40], ranges[50], ranges[60]]
            ### LEFT          obstacleMap[2]
            elif i == 2: current_direction_ranges = [ranges[70], ranges[80], ranges[90], ranges[100], ranges[110]]
            ### BACK-LEFT     obstacleMap[3]
            elif i == 3: current_direction_ranges = [ranges[120], ranges[130], ranges[140], ranges[150]]
            ### BACK          obstacleMap[4]
            elif i == 4: current_direction_ranges = [ranges[160], ranges[170], ranges[180], ranges[190], ranges[200]]
            ### BACK-RIGHT    obstacleMap[5]
            elif i == 5: current_direction_ranges = [ranges[210], ranges[220], ranges[230], ranges[240]]
            ### RIGHT         obstacleMap[6]
            elif i == 6: current_direction_ranges = [ranges[250], ranges[260], ranges[270], ranges[280], ranges[290]]
            ### FRONT-right   obstacleMap[7]
            elif i == 7: current_direction_ranges = [ranges[300], ranges[310], ranges[320], ranges[330]]

            arrSum = 0.0
            for j in range(len(current_direction_ranges)):
                if current_direction_ranges[j] == float('Inf'):
                    current_direction_ranges[j] = 3.5
                elif math.isnan(current_direction_ranges[j]):
                    current_direction_ranges[j] = 0
                else:
                    arrSum += current_direction_ranges[j]

            # If all 0s, laser data is not reliable. Return -1
            if arrSum == 0.0:
                obstacleMap[i] = -1
                continue

            current_direction_ranges=(list(filter(lambda num: num != 0.0, current_direction_ranges)))
                    
            # Else, check is the minimun ranges under safe stop distance 
            if min(current_direction_ranges) <= ALERT_DISTANCE:
                obstacleMap[i] = min(current_direction_ranges)
                #print(min(current_direction_ranges))
            else: 
                obstacleMap[i] = 0.0

        # Update the obstacle map
        self.obstacle_map = obstacleMap

        #print(self.obstacle_map[0])
    
    def update_goal(self, x, y):
        self.goal = [x,y]
        self.goal_reached=False

    def publish_vel(self):
        cmd = Twist()
        cmd.linear.x = self.linear_vel
        cmd.angular.z = self.angular_vel
        self.cmd_vel.publish(cmd)    

    def calculate_vel(self):

        obstacle_map = self.obstacle_map

        # Check if goal reached
        goal_dist = ((self.goal[0] - self.pos[0])**2 + (self.goal[1] - self.pos[1])**2)**0.5
        if goal_dist <= TOLERANCE: 
            self.goal_reached = True
            print("Arrived")
            self.angular_vel =0.0
            self.linear_vel = 0.0
            return

        # Calculate max attractive force        
        dx = self.goal[0] - self.pos[0]
        dy = self.goal[1] - self.pos[1]
        distance = sqrt(dx**2 + dy**2)
        '''
        if distance>=0.3:       max_att_force=0.2
        elif distance>=0.15:     max_att_force=0.1
        elif distance>=0.1:    max_att_force=0.05
        else:                   max_att_force=0.01
        '''

        # Get angle difference
        goal_angle = atan2(dy, dx)
        dtheta = goal_angle - self.orientation[2]
        if dtheta > math.pi or  dtheta < math.pi:
            dtheta =  mapRadianNegPiToPosPi(dtheta)

        #SUCCESS VER: unit rep force
        # Calculate unit attractive force - X: (-1,+1) Y: (0,+1)
        x_att_force_unit = max(min(dtheta / BURGER_MAX_ANGULAR_VEL, 1), -1)
        y_att_force_unit = max(min(distance / BURGER_MAX_LINEAR_VEL, 1), 0) * ((cos(dtheta)+1)/2)
        

        # Calculate repulsive forces for X Y
        x_rep_force, y_rep_force = 0, 0
        x_rep_matric = [0, 2**(-0.5), 1, 2**(-0.5), 0, -(2**(-0.5)), -1, -(2**(-0.5))]
        y_rep_matric = [-1, -(2**(-0.5)), 0, 2**(-0.5), 1, 2**(-0.5), 0, -(2**(-0.5))]
        x_rep_intensity_matric = [0, 2**(-0.5), 1, 2**(-0.5), 0, (2**(-0.5)), 1, (2**(-0.5))]
        y_rep_intensity_matric = [1, (2**(-0.5)), 0, 2**(-0.5), 1, 2**(-0.5), 0, (2**(-0.5))]
        sum_of_matrix = sum(x_rep_intensity_matric)
        
        '''
        obstacle_map = self.obstacle_map
        for i in range(8):
            if obstacle_map[i] == -1:
                obstacle_map[i] = 0
        
        k = (1)**2
        for i in range(8):
            if obstacle_map[i]!=0:
                obstacle_map[i]=k/(obstacle_map[i]**2)
        #print(obstacle_map)
                
        # X repulsive force
        for i in range(8):
            x_rep_force += obstacle_map[i] * x_rep_matric[i]
        x_rep_force = x_rep_force/8

        # Y repulsive force
        for i in range(8):
            y_rep_force += obstacle_map[i] * y_rep_matric[i]
        y_rep_force = y_rep_force/8
        '''
        # make the obstcle map become unit (0,1)
        obstacle_map_unit = obstacle_map
        print(obstacle_map)
        for i in range(8):
            obstacle_map_unit[i] = 1
            '''if obstacle_map_unit[i] <= 0.0:
                obstacle_map_unit[i] = 0.0
            elif obstacle_map_unit[i] <= SAFE_STOP_DISTANCE:
                obstacle_map_unit[i] = 1
            elif obstacle_map_unit[i] <= ALERT_DISTANCE:
                obstacle_map_unit[i] = (float(obstacle_map_unit[i]) - SAFE_STOP_DISTANCE) / (ALERT_DISTANCE - SAFE_STOP_DISTANCE)
            else:
                obstacle_map_unit[i] = 0.0'''

        #print(obstacle_map, obstacle_map_unit)
        
                        
        x_rep_force_unit, y_rep_force_unit, x_rep_force_intensity, y_rep_force_intensity = 0, 0, 0, 0
        # X repulsive force unit
        for i in range(8):
            x_rep_force_unit += obstacle_map_unit[i] * x_rep_matric[i]
            x_rep_force_intensity += obstacle_map_unit[i] * x_rep_intensity_matric[i]
        x_rep_force_unit = x_rep_force/sum_of_matrix*2
        x_rep_force_intensity = x_rep_force_intensity/sum_of_matrix

        # Y repulsive force unit
        for i in range(8):
            y_rep_force_unit += obstacle_map_unit[i] * y_rep_matric[i]
            y_rep_force_intensity += obstacle_map_unit[i] * y_rep_intensity_matric[i]
        y_rep_force_unit = y_rep_force/sum_of_matrix*2
        y_rep_force_intensity = y_rep_force_intensity/sum_of_matrix
        
        max_rep_distance = SAFE_STOP_DISTANCE
        repX = max_rep_distance * x_rep_force_unit
        repY = max_rep_distance * y_rep_force_unit

        attX = distance * cos(dtheta)
        attY = distance * sin(dtheta)

        # get a point between two point by ration
        resultX = attX * (1 - x_rep_force_intensity) + repX * x_rep_force_intensity
        resultY = attY * (1 - y_rep_force_intensity) + repY * y_rep_force_intensity

        resultDistance = (resultX**2 + resultY**2)**0.5
        if resultY < 0: resultDistance = -resultDistance
        if resultX == 0:
            resultAngle = 0
        else:
            resultAngle = atan2(resultY, resultX)
                
        x_force = max(min(resultAngle, BURGER_MAX_ANGULAR_VEL), -BURGER_MAX_ANGULAR_VEL)
        y_force = max(min(resultDistance, BURGER_MAX_LINEAR_VEL), 0) * ((cos(resultAngle)+1)/2)

        #1print(self.pos, resultX, resultY, resultDistance, resultAngle, x_force, y_force)

        # Combine forces
        #x_rep_force_max, y_rep_force_max = 30, 4
        '''
        x_force = (x_att_force * (x_rep_force_max-abs(x_rep_force))  + x_rep_force * abs(x_rep_force))/x_rep_force_max
        y_force = (y_att_force * (y_rep_force_max-abs(y_rep_force))  + y_rep_force * abs(y_rep_force))/y_rep_force_max
        '''
        '''# SUCCESS VER: unit att force
        x_force = BURGER_MAX_ANGULAR_VEL * x_att_force_unit
        y_force = BURGER_MAX_LINEAR_VEL * y_att_force_unit
        '''

        # Clipping


        # Calculate desired velocity
        self.angular_vel = x_force
        self.linear_vel = y_force


    def run(self):
                                            
        while not rospy.is_shutdown():
            if self.goal_reached:
                '''# set pose
                poseX = float(input("Enter the x coordinate of initial position: "))
                poseY = float(input("Enter the y coordinate of initial position: "))
                poseAngle = float(input("Enter the angle of initial position: "))
                self.update_initial_pose(poseX,poseY,poseAngle)'''
                
                #receive app command input
                x = float(input("Enter the x coordinate (or q to quit): "))
                if x == 'q':
                    break
                y = float(input("Enter the y coordinate: "))
                self.update_goal(x, y)
        
            self.calculate_vel()
            self.publish_vel()

            self.rate.sleep()
            


if __name__ == '__main__':

    try:
        tb = TurtleBot()
        tb.run()

    except rospy.ROSInterruptException:
        pass