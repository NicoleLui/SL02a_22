#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, sin, cos
import math
import time
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String, Bool, Float32
import string

# OBSTACLE DETECTION
STOP_DISTANCE = 0.25
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
LIDAR_SENSITIVITY = 1
WAIT_TIME = 60
avoidMode = ['STOP','NAIVE']

# TURTLEBOT MOVEMENT
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1


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
        self.xy_input = rospy.Subscriber('mobile_input_xy',Float32MultiArray,self.updateXY_input)
        rospy.Subscriber('image_identified_moving_thing_in_front', Bool, self.update_moving_thing_alert)
        self.rate = rospy.Rate(10)
        
        rospy.Subscriber('mobile_mode',String,self.getMode_callback)
        self.mode="manual"
        self.moving_thing_bool = False
        self.position = None
        self.orientation = None
        self.StartTime = 0
        self.frontObstacleDistance=-1
        self.inputX='o'
        self.inputY='o'

        #manual teleop
        rospy.Subscriber('mobile_input_dir',String,self.getKey_callback)
        self.mobileInput=''
        self.target_BURGER_MAX_LIN_VEL   = 0.0
        self.target_angular_vel  = 0.0
        self.control_BURGER_MAX_LIN_VEL  = 0.0
        self.control_angular_vel = 0.0
        self.turtlebot3_model = rospy.get_param("model", "burger")


        while not rospy.is_shutdown():
            if self.position is not None and self.orientation is not None:
                break
            rospy.sleep(0.1)


        rospy.loginfo("TurtleBot3 Position: {}".format(self.position))
        rospy.loginfo("TurtleBot3 Orientation: {}".format(self.orientation))

    def update_moving_thing_alert(self,msg):
        self.moving_thing_bool = msg.data
        return

    def update_odometry(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation

    def updateXY_input(self, msg):
        self.inputX=msg.data[0]
        self.inputY=msg.data[1]

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
                
            if self.moving_thing_bool==True:
                print("Dynamic obstacle detected, standby now...")
                cmd = Twist()
                cmd.angular.z =0.0
                cmd.linear.x = 0.0
            else:
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
                '''
                if self.obstacleMap[0]==1:
                    obsX=self.frontObstacleDistance*sin(theta)+self.position.x
                    obsY=self.frontObstacleDistance*cos(theta)+self.position.y
                    inList=False
                    for i in range (0,len(self.ObstaclePointList),2):
                        if obsX+SAFE_STOP_DISTANCE >= self.ObstaclePointList[i] and obsX-SAFE_STOP_DISTANCE <= self.ObstaclePointList[i] and obsY+SAFE_STOP_DISTANCE >= self.ObstaclePointList[
                            i+1] and obsY-SAFE_STOP_DISTANCE <= self.ObstaclePointList[i+1]:
                            inList=True
                            break
                    if not(inList):
                        msg=Float32MultiArray()
                        msg.data = [obsX,obsY]
                        self.ObstaclePointPub.publish(msg)
                '''

            #publish
            self.cmd_vel.publish(cmd)
            #run a while
            self.rate.sleep()
        rospy.loginfo("Reached target position ({}, {})".format(x, y))
        rospy.loginfo("Actual Position ({},{})".format(self.position.x,self.position.y))
        self.xy_input='oo'


    #manual move part

    def makeSimpleProfile(self, output, input, slop):
        if input > output:
            output = min( input, output + slop )
        elif input < output:
            output = max( input, output - slop )
        else:
            output = input

        return output

    def constrain(self, input, low, high):
        if input < low:
            input = low
        elif input > high:
            input = high
        else:
            input = input

        return input

    def checkLinearLimitVelocity(self,vel):
        if self.turtlebot3_model == "burger":
            vel = self.constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
        elif self.turtlebot3_model == "waffle" or self.turtlebot3_model == "waffle_pi":
            vel = self.constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
        else:
            vel = self.constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

        return vel

    def checkAngularLimitVelocity(self,vel):
        if self.turtlebot3_model == "burger":
            vel = self.constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
        elif self.turtlebot3_model == "waffle" or self.turtlebot3_model == "waffle_pi":
            vel = self.constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
        else:
            vel = self.constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
        
        return vel


    def getKey_callback(self, newMobileInput):
        self.mobileInput= newMobileInput.data
        print("newdata received",self.mobileInput)


    def getMode_callback(self, newMobileInput):
        self.mode= newMobileInput.data

        
    def manual_moveTo(self):
        if self.mobileInput == 'w' :
            self.target_BURGER_MAX_LIN_VEL = self.checkLinearLimitVelocity(self.target_BURGER_MAX_LIN_VEL + LIN_VEL_STEP_SIZE)

        elif self.mobileInput == 'x' :
            self.target_BURGER_MAX_LIN_VEL = self.checkLinearLimitVelocity(self.target_BURGER_MAX_LIN_VEL - LIN_VEL_STEP_SIZE)

        elif self.mobileInput == 'a' : 
            self.target_angular_vel = self.checkAngularLimitVelocity(self.target_angular_vel + ANG_VEL_STEP_SIZE)

        elif self.mobileInput == 'd' :
            self.target_angular_vel = self.checkAngularLimitVelocity(self.target_angular_vel - ANG_VEL_STEP_SIZE)

        elif self.mobileInput == ' ' or self.mobileInput == 's' :
            self.target_BURGER_MAX_LIN_VEL   = 0.0
            self.control_BURGER_MAX_LIN_VEL  = 0.0
            self. target_angular_vel  = 0.0
            self.control_angular_vel = 0.0

        else:
            if (self.mobileInput == '\x03'):
                cmd = Twist()
                cmd.angular.z =0.0
                cmd.linear.x = 0.0
                self.cmd_vel.publish(cmd)
                return

        self.mobileInput = ''

        self.getObstacleMap()
        self.target_BURGER_MAX_LIN_VEL, self.target_angular_vel = self.obstacleAvoidVel(self.target_BURGER_MAX_LIN_VEL,self.target_angular_vel,"STOP")
        if self.target_BURGER_MAX_LIN_VEL==0.0:
            self.control_BURGER_MAX_LIN_VEL  = 0.0
        if self.target_angular_vel==0.0:
            self.control_angular_vel=0.0

        twist = Twist()
        if self.moving_thing_bool==True:
                print("Dynamic obstacle detected, standby now...")
                twist.angular.z =0.0
                twist.linear.x = 0.0
                self.cmd_vel.publish(twist)
                return

        self.control_BURGER_MAX_LIN_VEL = self.makeSimpleProfile(self.control_BURGER_MAX_LIN_VEL, self.target_BURGER_MAX_LIN_VEL, (LIN_VEL_STEP_SIZE/2.0))
        twist.linear.x = self.control_BURGER_MAX_LIN_VEL; twist.linear.y = 0.0; twist.linear.z = 0.0

        self.control_angular_vel = self.makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = self.control_angular_vel
        self.cmd_vel.publish(twist)

    def run(self):
                                            
        while not rospy.is_shutdown():
            if self.mode=="shape":
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
            elif self.mode=="manual":
                self.manual_moveTo()

            


if __name__ == '__main__':

    #logging.basicConfig(filename=('log_moveToCoor'+str(datetime.now())+'.txt'), encoding='utf-8' , level=logging.DEBUG)

    try:
        tb = TurtleBot()
        tb.run()


    except rospy.ROSInterruptException:
        pass