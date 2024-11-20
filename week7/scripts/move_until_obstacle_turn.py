#!/usr/bin/python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

SAFE_DISTANCE = 0.5  

class MoveUntilObstacleTurn:
    def __init__(self):
        #Publisher for movement commands
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        #Subscribers for laser scan data and odometry
        self.scan_sub = rospy.Subscriber("/base_scan", LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        #States variables
        self.should_move = True
        self.turning = False
        self.start_yaw = None
        self.odom = None

    def odom_callback(self, msg):
        #Stores latest odometry message
        self.odom = msg

    def get_yaw(self):
        #Gets current yaw (orientation) from odometry
        orientation_q = self.odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def scan_callback(self, data):
        #Processes scan data to detect obstacles directly in front
        front_ranges = data.ranges[len(data.ranges) // 2 - 10 : len(data.ranges) // 2 + 10]
        closest_front_distance = min(front_ranges)

        rospy.loginfo(f"Closest distance to obstacle in front: {closest_front_distance:.2f} meters")

        #Determines if an obstacle is within safe distance
        if closest_front_distance < SAFE_DISTANCE:
            if self.should_move:
                rospy.loginfo("Obstacle detected! Starting to turn.")
                self.should_move = False
                self.turning = True
                self.start_yaw = self.get_yaw()
        else:
            #Resumes moving if path is clear and no turn in progress
            if not self.turning:
                self.should_move = True

    def move(self):
        #Create Twist message
        move_cmd = Twist()

        if self.should_move:
            #Move forward
            move_cmd.linear.x = 0.2
            move_cmd.angular.z = 0.0
            rospy.loginfo("Moving forward.")
        elif self.turning:
            #Start turning if an obstacle is detected
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.5  #Turn at 0.5 rad/s
            rospy.loginfo("Turning.")

            #Check if we've turned 1 radian from the start yaw
            current_yaw = self.get_yaw()
            turn_distance = abs(current_yaw - self.start_yaw)
            if turn_distance > 1.0:
                #Stop turning after completing a 1 radian turn
                move_cmd.angular.z = 0.0
                self.turning = False
                self.should_move = True
                rospy.loginfo("Completed turn. Resuming forward motion.")

        #Publish the command
        self.pub.publish(move_cmd)

if __name__ == '__main__':
    rospy.init_node('move_until_obstacle')
    node = MoveUntilObstacleTurn()
    
    #Set a loop rate for regular checks
    rate = rospy.Rate(10)  # 10 Hz

    #Main loop
    while not rospy.is_shutdown():
        node.move()
        rate.sleep()
