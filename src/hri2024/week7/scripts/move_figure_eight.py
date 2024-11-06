#!/usr/bin/python3
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class MoveFigureEight:
    def __init__(self):
        rospy.init_node('move_figure_eight')

        #Initialize publisher and subscriber
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.odom = Odometry()
        rospy.sleep(rospy.Duration(0.5))

    def odom_callback(self, msg):
        self.odom = msg

    def get_yaw(self):
        orientation_q = self.odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def move_in_circle(self, direction=1, radius=1.5, speed=0.5):
        #Move in a circle in a given direction
        #direction: 1 for counterclockwise, -1 for clockwise
        #radius: Radius of the circle
        #speed: Linear speed of the robot
        t = Twist()
        t.linear.x = speed
        t.angular.z = direction * speed / radius
        start_yaw = self.get_yaw()
        prev_yaw = start_yaw
        cumulative_angle = 0
        rate = rospy.Rate(10)  # Set the rate to 10 Hz

        while not rospy.is_shutdown():
            self.pub.publish(t)

            #Calculates current yaw and accumulated rotation
            cur_yaw = self.get_yaw()
            delta_yaw = (cur_yaw - prev_yaw + math.pi) % (2 * math.pi) - math.pi
            cumulative_angle += abs(delta_yaw)
            prev_yaw = cur_yaw

            #Stops after completing a full circle
            if cumulative_angle >= 2 * math.pi:
                t.linear.x = 0.0
                t.angular.z = 0.0
                self.pub.publish(t)
                break

            rate.sleep()

    def move_in_figure_eight(self):
        #Moves in a counterclockwise circle for the first loop of the figure eight
        self.move_in_circle(direction=1)
        rospy.sleep(1)  #Small pause between loops

        #Moves in a clockwise circle for the second loop of the figure eight
        self.move_in_circle(direction=-1)

if __name__ == '__main__':
    move_figure_eight = MoveFigureEight()
    move_figure_eight.move_in_figure_eight()
