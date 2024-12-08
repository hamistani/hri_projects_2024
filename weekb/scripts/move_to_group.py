#!/usr/bin/env python3
import rospy
from people_msgs.msg import People
from geometry_msgs.msg import Twist

class MoveToGroup:
    def __init__(self):
        rospy.init_node('move_to_group')
        self.people_sub = rospy.Subscriber('/robot_0/detected_groups', People, self.people_callback)
        self.cmd_pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
        self.target = None
        rospy.loginfo("Move to Group Node Initialized")

    def people_callback(self, msg):
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
            if self.target:
                rospy.loginfo(f"Moving to target: {self.target}")
                cmd = Twist()
                cmd.linear.x = 0.2  
                cmd.angular.z = 0.0  
                self.cmd_pub.publish(cmd)
                self.target = None  
            else:
                rospy.loginfo("No target set, waiting for group detection.")
            rate.sleep()

if __name__ == "__main__":
    mover = MoveToGroup()
    mover.move_to_target()
