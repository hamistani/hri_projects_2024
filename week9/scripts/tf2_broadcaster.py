#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
import math

#Variables for robot state
robot_pos = geometry_msgs.msg.Point()
robot_orientation = 0.0
scan_ranges = []

#Laser scan callback to store data
def laser_callback(msg):
    global scan_ranges
    scan_ranges = msg.ranges
    rospy.loginfo(f"Laser data received. First few ranges: {scan_ranges[:5]}")  #Debugging laser data

#Odometry callback to update robot position and orientation
def odom_callback(msg):
    global robot_pos, robot_orientation
    robot_pos = msg.pose.pose.position
    quaternion = [
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
    ]
    _, _, robot_orientation = transformations.euler_from_quaternion(quaternion)

#Function to detect the closest "leg-like" object
def get_leg_pos_and_angle():
    global scan_ranges

    if not scan_ranges:
        return None, None

    #Finds the closest laser scan range
    closest_range = min(scan_ranges)
    if closest_range > 5.0:  # Ignore detections beyond 5 meters (adjust as needed)
        return None, None

    closest_index = scan_ranges.index(closest_range)

    #Calculates the angle to the detected leg
    angle_to_leg = closest_index * 0.01 - math.pi 

    #Calculates leg position in the robot's coordinate frame
    leg_pos = geometry_msgs.msg.Point()
    leg_pos.x = closest_range * math.cos(angle_to_leg)
    leg_pos.y = closest_range * math.sin(angle_to_leg)
    leg_pos.z = 0.0

    rospy.loginfo(f"Leg detected at position: ({leg_pos.x:.2f}, {leg_pos.y:.2f}) with angle {angle_to_leg:.2f} radians")
    return leg_pos, angle_to_leg

#Main broadcaster function to publish the detected leg transform
def broadcaster():
    rospy.init_node("leg_tf_broadcaster")
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10)

    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/base_scan", LaserScan, laser_callback)

    while not rospy.is_shutdown():
        leg_pos, leg_angle = get_leg_pos_and_angle()

        if leg_pos is None:
            rospy.loginfo("No valid leg detected")
            rate.sleep()
            continue

        #Transforms Message
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_laser_link"  
        t.child_frame_id = "detected_leg"      

        #Adjusts position of the detected leg
        t.transform.translation.x = leg_pos.x
        t.transform.translation.y = leg_pos.y
        t.transform.translation.z = leg_pos.z

        #Converts Euler angles to quaternion for rotation
        q = transformations.quaternion_from_euler(0, 0, leg_angle)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        rospy.loginfo(f"Broadcasting leg position: {leg_pos.x:.2f}, {leg_pos.y:.2f}")
        tf_broadcaster.sendTransform(t)
        rate.sleep()

if __name__ == "__main__":
    try:
        broadcaster()
    except rospy.ROSInterruptException:
        pass
