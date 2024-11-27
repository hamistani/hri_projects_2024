#!/usr/bin/python3
import rospy
import math
import time
import tf2_ros
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from functools import partial
import geometry_msgs.msg
import copy

current_joint_states = None

def joint_state_callback(msg):
    global current_joint_states
    rospy.loginfo("Received message on /joint_states_input")
    current_joint_states = msg

def move_arm(pub):
  arm_joint_states = JointState()
  arm_joint_states.header = Header()
  arm_joint_states.name = ["LElbowRoll"]
  base_time = time.time()

  while not rospy.is_shutdown():
    current_time = time.time() - base_time
    arm_joint_states.position = [0.2 * math.cos(current_time)]

    arm_joint_states.header.stamp = rospy.get_rostime()
    pub.publish(arm_joint_states)
    rospy.sleep(0.1)

def timer_callback(event, pub):
    move_arm(pub)

def main():
    global current_joint_states
    rospy.init_node('look_at_hand')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pub = rospy.Publisher('joint_states_out', JointState, queue_size=10)

    rospy.Subscriber('joint_states', JointState, joint_state_callback)

    rate = rospy.Rate(10.0)  

    angle_threshold = 0.01  
    head_movement = 0.5  
    yaw_limit = (-1.57, 1.57)
    pitch_limit = (0.0, 1.0)

    previous_yaw_angle = 0.0
    previous_pitch_angle = 0.0

    rospy.Timer(rospy.Duration(0.1), partial(timer_callback, pub=pub))

    while current_joint_states is None and not rospy.is_shutdown():
      rospy.loginfo(":)"*30) 

    while not rospy.is_shutdown():
        updated_joint_states = JointState()
        updated_joint_states.header = Header()
        updated_joint_states.header.stamp = rospy.get_rostime()
        updated_joint_states.name = copy.deepcopy(current_joint_states.name)
        updated_joint_states.position = copy.deepcopy(current_joint_states.position)
        rospy.loginfo(updated_joint_states.name)
        try:
            trans = tfBuffer.lookup_transform('Head', 'l_gripper', rospy.Time(), rospy.Duration(1.0))

            rospy.loginfo(f"Hand position: {trans.transform.translation.x}, {trans.transform.translation.y}, {trans.transform.translation.z}")
            
            yaw_angle = math.atan2(trans.transform.translation.y, trans.transform.translation.x)
            pitch_angle = math.atan2(trans.transform.translation.z, trans.transform.translation.x)
            
            rospy.loginfo(f"Calculated yaw: {yaw_angle}, pitch: {pitch_angle}")

            yaw_diff = abs(yaw_angle - previous_yaw_angle)
            pitch_diff = abs(pitch_angle - previous_pitch_angle)
            
            if yaw_diff > angle_threshold or pitch_diff > angle_threshold:
                rospy.loginfo(f"Moving head to yaw {yaw_angle}, pitch {pitch_angle}")

                smoothed_yaw = previous_yaw_angle + head_movement * (yaw_angle - previous_yaw_angle)
                smoothed_pitch = previous_pitch_angle + head_movement * (pitch_angle - previous_pitch_angle)

                smoothed_yaw = max(min(smoothed_yaw, yaw_limit[1]), yaw_limit[0])
                smoothed_pitch = max(min(smoothed_pitch, pitch_limit[1]), pitch_limit[0])

                #Tuple
                updated_joint_states.name = ("HeadYaw", "HeadPitch")
                updated_joint_states.position = (smoothed_yaw, smoothed_pitch)

                previous_yaw_angle = smoothed_yaw
                previous_pitch_angle = smoothed_pitch

                pub.publish(updated_joint_states)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to lookup transform between 'Head' and 'l_gripper'")
            pub.publish(updated_joint_states)


        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
