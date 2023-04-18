#!/usr/bin/env python
import rospy

# Import map_server package
from map_server import MapServer

if __name__ == '__main__':
    rospy.init_node('tb_map_server')

    # Set the robot name and map file path
    robot_name = 'tb104'
    map_file = '/path/to/map.yaml'

    # Set the robot parameters for map_server
    rospy.set_param('/{}/map_server/use_map_topic'.format(robot_name), False)
    rospy.set_param('/{}/map_server/frame_id'.format(robot_name), 'map')
    rospy.set_param('/{}/map_server/always_send_full_map'.format(robot_name), True)
    rospy.set_param('/{}/map_server/params/yaml_filename'.format(robot_name), map_file)

    # Start the map_server
    map_server = MapServer(robot_name)
    map_server.start()
