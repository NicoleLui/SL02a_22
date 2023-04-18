#!/usr/bin/env python
import rospy

# Import amcl package
from amcl import AMCL

if __name__ == '__main__':
    rospy.init_node('tb_amcl')

    # Set the robot name
    robot_name = 'tb101'

    # Set the robot parameters for amcl
    rospy.set_param('/{}/amcl/odom_model_type'.format(robot_name), 'diff')
    rospy.set_param('/{}/amcl/update_min_d'.format(robot_name), 0.2)
    rospy.set_param('/{}/amcl/update_min_a'.format(robot_name), 0.1)

    # Start the amcl
    amcl = AMCL(robot_name)
    amcl.start()
