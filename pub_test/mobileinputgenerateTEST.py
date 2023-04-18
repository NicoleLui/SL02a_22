#!/usr/bin/env python2

import rospy
from std_msgs.msg import String

if __name__=="__main__":
    rospy.init_node('turtlebot3_teleopavoid')
    pub=rospy.Publisher("mobile_input_xy",String,queue_size=10)
    try:
        while not rospy.is_shutdown():
            pub.publish(["1","0"])
            rospy.sleep(10)
    except rospy.ROSInterruptException:
        pass