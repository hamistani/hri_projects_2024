#!/usr/bin/python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import tf2_ros
import geometry_msgs.msg

SAFE_DISTANCE = 0.5  # Safe distance to obstacles
# PERSON_DETECTION_THRESHOLD = 1.0  # Distance threshold for detecting person

class MoveTowardPersonWithObstacle:
    def __init__(self):
        #Publisher for movement commands
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        #Subscribers for laser scan data, odometry, and person detection (TF)
        self.scan_sub = rospy.Subscriber("/base_scan", LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        #States variables
        self.should_move = True
        self.turning = False
        self.start_yaw = None
        self.odom = None
        self.person_pos = None

    def odom_callback(self, msg):
        #Stores the latest odometry message
        self.odom = msg

    def get_yaw(self):
        #Gets the current yaw (orientation) from odometry
        orientation_q = self.odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        _, _, yaw = euler_from_quaternion(orientation_list)
        return yaw

    def scan_callback(self, data):
        #Processes to detect obstacles directly in front
        front_ranges = data.ranges[len(data.ranges) // 2 - 10 : len(data.ranges) // 2 + 10]
        closest_front_distance = min(front_ranges)

        # rospy.loginfo(f"Closest distance to obstacle in front: {closest_front_distance:.2f} meters")

        #Determines if an obstacle is within a safe distance
        if closest_front_distance < SAFE_DISTANCE:
            if self.should_move:
                rospy.loginfo("Obstacle detected! Starting to turn.")
                self.should_move = False
                self.turning = True
                self.start_yaw = self.get_yaw()
        elif self.person_pos and self.person_pos.x < SAFE_DISTANCE:
            rospy.loginfo("Detected leg is too close. Stopping.")
            self.should_move = False
            self.turning = False
        else:
            self.should_move = True
            
            # #Resumes moving if the path is clear and no turn is in progress
            # if not self.turning:
            #     self.should_move = True

    def get_person_pos(self):
        try:
            #Looks up the transform from base_link to detected_leg
            transform = self.tf_buffer.lookup_transform("base_link", "detected_leg", rospy.Time(0), rospy.Duration(1.0))
            self.person_pos = transform.transform.translation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.person_pos = None

    def move(self):
        #Creates Twist message for robot movement
        move_cmd = Twist()

        if self.should_move and self.person_pos:
            #Calculates the angle to the person (goal)
            goal_x = self.person_pos.x
            goal_y = self.person_pos.y

            #Calculates the angle to the goal (person) relative to the robot's pos
            angle_to_goal = math.atan2(goal_y - self.odom.pose.pose.position.y, goal_x - self.odom.pose.pose.position.x)

            #Gets the current yaw (orientation) of the robot
            current_yaw = self.get_yaw()

            #Calculates the difference between the current orientation and the goal direction
            angle_diff = angle_to_goal - current_yaw

            #Normalizes the angle difference to the range [-pi, pi]
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            rospy.loginfo(f"Angle difference to goal: {angle_diff:.2f}")

            #If the robot is not facing the goal, turn toward it
            if abs(angle_diff) > 0.1: 
                move_cmd.linear.x = 0.0 
                move_cmd.angular.z = 0.5 * angle_diff / abs(angle_diff)  # Turn towards the goal
                #rospy.loginfo(f"Turning towards goal. Angular speed: {move_cmd.angular.z:.2f}")
            else:
                #If the robot is facing the goal, move forward
                move_cmd.linear.x = 0.2
                move_cmd.angular.z = 0.0
                #rospy.loginfo("Moving toward goal.")

        elif self.turning:
            #Starts turning if an obstacle is detected
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.5  # Turn at 0.5 rad/s
            #rospy.loginfo("Turning.")

            #Checks if turned 1 radian from the start yaw
            current_yaw = self.get_yaw()
            turn_distance = abs(current_yaw - self.start_yaw)
            if turn_distance > 1.0:
                #Stops turning after completing a 1 radian turn
                move_cmd.angular.z = 0.0
                self.turning = False
                self.should_move = True
                #rospy.loginfo("Completed turn. Resuming forward motion.")

        self.pub.publish(move_cmd)

if __name__ == '__main__':
    rospy.init_node('move_toward_person_with_obstacle_avoidance')
    node = MoveTowardPersonWithObstacle()

    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        node.get_person_pos() 
        node.move()
        rate.sleep()
