import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance')
        rospy.Subscriber('scan', LaserScan, self.scan_callback)
        rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.dist_threshold = 0.5
        self.goal = [0, 0]
        self.pos = [0, 0, 0]
        self.vel = [0, 0, 0]
        self.angular_vel = 0
        self.linear_vel = 0

        self.goal_reached = True
        self.STOP_DISTANCE = 0.01

    def get_new_goal(self):
        x = float(input("Enter the x coordinate (or q to quit): "))
        if x == 'q':
            print("cancel input.")
            return
        y = float(input("Enter the y coordinate: "))

        self.goal_reached = False
        self.goal = [x,y]
        return 
    
    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
        scan_filter = scan.ranges

        for i in range(len(scan_filter)):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0

        return scan_filter
    
    def get_odom(self):
        return rospy.wait_for_message('odom', Odometry)

    def scan_callback(self, scan_data):
        front = scan_data.ranges[0]
        back = scan_data.ranges[180]
        left = scan_data.ranges[90]
        right = scan_data.ranges[270]

        # Calculate repulsive forces
        k = 1
        
        rep_force_left = 0
        rep_force_right = 0
        rep_force_front = 0
        rep_force_back = 0
        if left != 0:
            rep_force_left = k / (left**2)
        if right != 0:
            rep_force_right = k / (right**2)
        if front != 0:
            rep_force_front = k / (front**2)
        if back != 0:
            rep_force_back = k / (back**2)

        # Calculate attractive force
        goal_dist = ((self.goal[0] - self.pos[0])**2 + (self.goal[1] - self.pos[1])**2)**0.5
        att_force = 0.5 * goal_dist

        print("Position: ",self.pos[0],self.pos[1])

        # Check is goad reached
        if goal_dist < self.STOP_DISTANCE:
            self.goal_reached = True
            return

        # Combine forces
        x_forc  = att_force * (self.goal[0] - self.pos[0]) / goal_dist - rep_force_left + rep_force_right
        y_force = att_force * (self.goal[1] - self.pos[1]) / goal_dist - rep_force_front +rep_force_back

        # Calculate desired velocity
        self.angular_vel = -x_force / 2
        self.linear_vel = y_force

    def odom_callback(self, odom_data):
        x = odom_data.pose.pose.position.x
        y = odom_data.pose.pose.position.y
        orientation = odom_data.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        self.pos = [x, y, yaw]
        self.vel = [odom_data.twist.twist.linear.x, odom_data.twist.twist.linear.y, odom_data.twist.twist.angular.z]



    def run(self):

        while not rospy.is_shutdown():

            if self.goal_reached:
                print("POSITION REACHED")
                print("Position: ",self.pos[0],self.pos[1])
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                self.get_new_goal()


            #1
            # 1self.odom_callback(self.get_odom())
            #self.scan_callback(self.get_scan())

            cmd = Twist()
            cmd.linear.x = self.linear_vel
            cmd.angular.z = self.angular_vel
            self.cmd_pub.publish(cmd)
            self.rate.sleep()

if __name__ == '__main__':
    oa = ObstacleAvoidance()
    oa.run()
