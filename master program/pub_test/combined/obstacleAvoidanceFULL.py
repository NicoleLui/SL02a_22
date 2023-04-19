#!/usr/bin/env python2

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#www
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
import math
from sensor_msgs.msg import LaserScan


# TURTLEBOT MOVEMENT
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

# OBSTACLE DETECTION
STOP_DISTANCE = 0.25
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
LIDAR_SENSITIVITY = 1
avoidMode = ['STOP','TURN']


msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

# obstacle
class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.obstacleMap = [0,0,0,0,0,0,0,0]
        self.oldeMap = [0,0,0,0,0,0,0,0]
        # self.
        #self.obstacle()


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
    '''
    def obstacle(self):
        twist = Twist()
        turtlebot_moving = True

        #while not rospy.is_shutdown():
        lidar_distances = self.get_scan()
        # hack for 0 value error  in /slam.range
        min_distance = min(list(filter(lambda num: num != 0.0, lidar_distances)))

        if min_distance < SAFE_STOP_DISTANCE:
            if turtlebot_moving:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self._cmd_pub.publish(twist)
                self.turtlebot_moving = False
                rospy.loginfo('Stop: %f', min_distance)
        else:
            twist.linear.x = BURGER_MAX_LIN_VEL
            twist.angular.z = 0.0
            self._cmd_pub.publish(twist)
            self.turtlebot_moving = True
            rospy.loginfo('Distance of the obstacle : %f', min_distance)
    '''

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
        oldMap = self.oldeMap
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

        elif avoidMode == 'TURN':
            # condition any of obstacle sector 0,1,7 == 1 && burger moving forward
            if ((obstacleMap[0]==1 or obstacleMap[1]==1 or obstacleMap[7]==1 ) and oldLinearVel>0):
                newLinearVel=0.0
                if oldAngularVel>0:
                    newAngularVel=-0.5
                elif oldAngularVel<0:
                    newAngularVel=0.5
                elif (obstacleMap[7]==1 or obstacleMap[6]==1): #sth on right,turn slightly left
                    newAngularVel=0.5
                else : newAngularVel=-0.5

                cmd = Twist()
                cmd.linear.x ,cmd.angular.z = newLinearVel,newAngularVel
                self.cmd_vel.publish(cmd)
                rospy.sleep(0.1)

            # condition any of obstacle sector 3,4,5 == 1 && burger moving backward
            elif ((obstacleMap[3]==1 or obstacleMap[4]==1 or obstacleMap[5]==1) and oldLinearVel<0):
                newLinearVel=0.0
                if oldAngularVel>0:
                    newAngularVel=-0.5
                elif oldAngularVel<0:
                    newAngularVel=0.5
                elif (obstacleMap[3]==1 or obstacleMap[2]==1): #sth on left,turn slightly right
                    newAngularVel=-0.5
                else : newAngularVel=0.5
                
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
            

        return newLinearVel, newAngularVel

def getKey_callback(newMobileInput):
    global mobileInput
    mobileInput= newMobileInput.data

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

def vels(target_BURGER_MAX_LIN_VEL, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_BURGER_MAX_LIN_VEL,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('turtlebot3_teleavoid')
    
    #rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('mobile_input',String,getKey_callback)

    turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    target_BURGER_MAX_LIN_VEL   = 0.0
    target_angular_vel  = 0.0
    control_BURGER_MAX_LIN_VEL  = 0.0
    control_angular_vel = 0.0
    mobileInput=''

    try:
        print(msg)
        obstacle = Obstacle()

        while (1):

            
            if mobileInput == 'w' :
                target_BURGER_MAX_LIN_VEL = checkLinearLimitVelocity(target_BURGER_MAX_LIN_VEL + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_BURGER_MAX_LIN_VEL,target_angular_vel))
            elif mobileInput == 'x' :
                target_BURGER_MAX_LIN_VEL = checkLinearLimitVelocity(target_BURGER_MAX_LIN_VEL - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_BURGER_MAX_LIN_VEL,target_angular_vel))
            elif mobileInput == 'a' : 
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_BURGER_MAX_LIN_VEL,target_angular_vel))
            elif mobileInput == 'd' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_BURGER_MAX_LIN_VEL,target_angular_vel))
            elif mobileInput == ' ' or mobileInput == 's' :
                target_BURGER_MAX_LIN_VEL   = 0.0
                control_BURGER_MAX_LIN_VEL  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print(vels(target_BURGER_MAX_LIN_VEL, target_angular_vel))
            else:
                if (mobileInput == '\x03'):
                    break

            if status == 20 :
                print(msg)
                status = 0

            obstacle.getObstacleMap()
            target_BURGER_MAX_LIN_VEL, target_angular_vel = obstacle.obstacleAvoidVel(target_BURGER_MAX_LIN_VEL,target_angular_vel)
            if target_BURGER_MAX_LIN_VEL==0.0:
                control_BURGER_MAX_LIN_VEL  = 0.0
            if target_angular_vel==0.0:
                control_angular_vel=0.0

            twist = Twist()

            control_BURGER_MAX_LIN_VEL = makeSimpleProfile(control_BURGER_MAX_LIN_VEL, target_BURGER_MAX_LIN_VEL, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_BURGER_MAX_LIN_VEL; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
