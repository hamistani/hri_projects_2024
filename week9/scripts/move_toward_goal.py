#!/usr/bin/python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import tf2_ros

SAFE_DISTANCE = 0.5  
STOP_DISTANCE = 0.7 
ATTRACTIVE_GAIN = 1.0  
REPULSIVE_GAIN = 2.0  
REPULSIVE_RADIUS = 1.0  
MAX_LINEAR_SPEED = 0.2 
MAX_ANGULAR_SPEED = 0.5  

class NavigateToLegs:
    def __init__(self):
        #Publisher for movement commands
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        #Subscribers for laser scan and odometry
        self.scan_sub = rospy.Subscriber("/base_scan", LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        #TF listener for detected legs
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # State variables
        self.person_pos = None #Detected leg position
        self.odom = None #Robot's odometry data
        self.repulsive_force = (0, 0) #Repulsive force from obstacles
        self.searching = True #Robot is in searching mode

    def odom_callback(self, msg):
        #Stores the latest odometry message
        self.odom = msg

    def get_yaw(self):
        #Gets the current yaw (orientation) from odometry
        orientation_q = self.odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def scan_callback(self, data):
        #Computes the repulsive force from obstacles
        self.repulsive_force = (0, 0)
        for i, distance in enumerate(data.ranges):
            if distance < REPULSIVE_RADIUS and not math.isinf(distance):
                angle = data.angle_min + i * data.angle_increment
                force_magnitude = REPULSIVE_GAIN * (1 / distance - 1 / REPULSIVE_RADIUS) / (distance**2)
                self.repulsive_force = (
                    self.repulsive_force[0] - force_magnitude * math.cos(angle),
                    self.repulsive_force[1] - force_magnitude * math.sin(angle)
                )

    def get_person_pos(self):
        try:
            #Looks up the transform from base_link to detected_leg
            transform = self.tf_buffer.lookup_transform("base_link", "detected_leg", rospy.Time(0), rospy.Duration(1.0))
            self.person_pos = transform.transform.translation
            self.searching = False  # Legs found, stop searching
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.person_pos = None
            self.searching = True  #Continues searching for legs

    def compute_attractive_force(self):
        #Computes the attractive force toward the detected legs
        if self.person_pos:
            distance = math.sqrt(self.person_pos.x**2 + self.person_pos.y**2)
            force_magnitude = ATTRACTIVE_GAIN * distance
            angle = math.atan2(self.person_pos.y, self.person_pos.x)
            return (
                force_magnitude * math.cos(angle),
                force_magnitude * math.sin(angle)
            )
        return (0, 0)

    def move(self):
        move_cmd = Twist()

        if self.searching:
            #Searching mode: Rotate in place
            rospy.loginfo("Searching for legs...")
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = MAX_ANGULAR_SPEED * 0.5  # Rotate slowly
        else:
            #Navigation mode: Combine forces and navigate
            attractive_force = self.compute_attractive_force()
            total_force = (
                attractive_force[0] + self.repulsive_force[0],
                attractive_force[1] + self.repulsive_force[1]
            )

            total_magnitude = math.sqrt(total_force[0]**2 + total_force[1]**2)
            if total_magnitude > 0:
                angle = math.atan2(total_force[1], total_force[0])
                current_yaw = self.get_yaw()
                angle_diff = angle - current_yaw

                #Normalize angle difference
                angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

                #Set velocities
                if math.sqrt(self.person_pos.x**2 + self.person_pos.y**2) > STOP_DISTANCE:
                    move_cmd.linear.x = min(MAX_LINEAR_SPEED, total_magnitude)  # Cap linear speed
                    move_cmd.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, 2 * angle_diff))
                else:
                    rospy.loginfo("Reached the legs. Stopping.")
                    move_cmd.linear.x = 0.0
                    move_cmd.angular.z = 0.0
            else:
                rospy.loginfo("No forces detected. Searching...")
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = MAX_ANGULAR_SPEED * 0.5

        #Publishes the movement command
        self.pub.publish(move_cmd)

if __name__ == '__main__':
    rospy.init_node('navigate_to_legs')
    node = NavigateToLegs()

    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        node.get_person_pos()  #Updates the detected leg position
        node.move()            
        rate.sleep()
