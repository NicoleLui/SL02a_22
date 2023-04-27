#!/usr/bin/env python2

import rospy
from std_msgs.msg import String

import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

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

if __name__=="__main__":
    '''
    rospy.init_node('turtlebot3_teleopavoid')
    pub=rospy.Publisher("mobile_input_xy",String,queue_size=10)
    try:
        while not rospy.is_shutdown():
            pub.publish(["1","0"])
            rospy.sleep(10)
    except rospy.ROSInterruptException:
        pass
    '''


    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('turtlebot3_teleopavoid')

    pub=rospy.Publisher("mobile_input_dir",String,queue_size=2)

    try:
        key = ''
        while not rospy.is_shutdown():
            while key == '':
                key = getKey()
            if (key == '\x03'):
                    break
            msg = String()
            msg.data = key
            pub.publish(msg)
            key = ''
    except rospy.ROSInterruptException:
        pass

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)