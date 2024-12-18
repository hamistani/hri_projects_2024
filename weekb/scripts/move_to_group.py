#!/usr/bin/env python3
import rospy
from people_msgs.msg import People
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class MoveToGroup:
    def __init__(self):
        rospy.init_node('move_to_group')
        self.people_sub = rospy.Subscriber('/robot_0/detected_groups', People, self.people_callback)
        self.cmd_pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/robot_0/odom', Odometry, self.odom_callback)
        self.target = None
        self.robot_position = None
        self.robot_yaw = None
        self.reached_target = False
        rospy.loginfo("Move to Group Node Initialized")

    def odom_callback(self, msg):
        self.robot_position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

    def people_callback(self, msg):
        if self.reached_target:
            return  
        rospy.loginfo(f"Received People message with {len(msg.people)} people")
        for person in msg.people:
            if "circle" in person.name:
                rospy.loginfo("Targeting circle group")
                self.target = self.get_circle_center(msg.people)
                break
            elif "line" in person.name:
                rospy.loginfo("Targeting line group")
                self.target = self.get_line_end(msg.people)
                break

    def get_circle_center(self, people):
        x, y, count = 0, 0, 0
        for person in people:
            x += person.position.x
            y += person.position.y
            count += 1
        center = (x / count, y / count)
        rospy.loginfo(f"Circle center calculated: {center}")
        return center

    def get_line_end(self, people):
        end = (people[-1].position.x, people[-1].position.y)
        rospy.loginfo(f"Line end calculated: {end}")
        return end

    def move_to_target(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.target and self.robot_position:
                target_x, target_y = self.target
                dx = target_x - self.robot_position.x
                dy = target_y - self.robot_position.y

                # Distance to the target
                distance = math.sqrt(dx**2 + dy**2)

                # Angle to the target
                target_angle = math.atan2(dy, dx)
                angle_diff = target_angle - self.robot_yaw
                angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # Normalize angle

                rospy.loginfo(f"Distance to target: {distance}, Angle diff: {angle_diff}")

                # Stop if close to the target
                if distance < 0.5:
                    rospy.loginfo("Reached the target. Stopping.")
                    self.stop()
                    self.reached_target = True
                    self.target = None
                    continue

                # Command movement
                cmd = Twist()
                if abs(angle_diff) > 0.1:  # Turn toward the target
                    cmd.angular.z = max(-0.5, min(0.5, angle_diff))  # Cap angular speed
                else:
                    cmd.linear.x = max(0.1, min(0.2, distance))  # Cap linear speed
                self.cmd_pub.publish(cmd)
            else:
                rospy.loginfo("No target set, waiting for group detection.")
            rate.sleep()

    def stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)  # Publish zero velocity

if __name__ == "__main__":
    mover = MoveToGroup()
    mover.move_to_target()
