#!/usr/bin/env python3

import rospy 
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
import math

#Stores last timestamp to avoid redundant TF broadcasting 
last_timestamp = None

#Variable for robot position (x, y, theta)
robot_pos = geometry_msgs.msg.Point()
robot_orientation = 0.0

def odom_callback(msg):
    global robot_pos, robot_orientation
    robot_pos = msg.pose.pose.position

    quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    _, _, robot_orientation = transformations.euler_from_quaternion(quaternion)

def get_leg_pos_and_angle():
    leg_pos = geometry_msgs.msg.Point()
    leg_pos.x = robot_pos.x + 1.0
    leg_pos.y = robot_pos.y + 1.0
    leg_pos.z = 0.0

    leg_angle = robot_orientation

    return leg_pos, leg_angle

def broadcaster():
    rospy.init_node('leg_tf_broadcaster')    
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10)

    rospy.Subscriber('/odom', Odometry, odom_callback)

    while not rospy.is_shutdown():
        leg_pos, leg_angle = get_leg_pos_and_angle()

        #Transform Message
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.child_frame_id = "detected_leg"

        #Adjust pos of detected leg
        t.transform.translation.x = leg_pos.x
        t.transform.translation.y = leg_pos.y
        t.transform.translation.z = leg_pos.z

        #Convert Euler angles to quaternion
        q = transformations.quaternion_from_euler(0, 0, leg_angle)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        tf_broadcaster.sendTransform(t)
        rate.sleep()

if __name__ == '__main__':
    try: 
        broadcaster()
    except rospy.ROSInterruptException:
        pass   