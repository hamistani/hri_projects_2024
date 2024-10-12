#!/usr/bin/python3
import rospy
import math
import tf2_ros
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

current_joint_states = JointState()

def joint_state_callback(msg):
    global current_joint_states
    current_joint_states = msg

def main():
    rospy.init_node('tf2_look_at_hand')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    rospy.Subscriber('/joint_states_input', JointState, joint_state_callback)

    rate = rospy.Rate(10.0)  

    angle_threshold = 0.01  

    head_movement = 0.5  

    previous_yaw_angle = 0.0
    previous_pitch_angle = 0.0

    yaw_limit = (-1.57, 1.57)  
    pitch_limit = (-1.0, 1.0)

    while not rospy.is_shutdown():
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

                updated_joint_states = JointState()
                updated_joint_states.header = Header()
                updated_joint_states.header.stamp = rospy.get_rostime()
                updated_joint_states.name = current_joint_states.name[:]
                updated_joint_states.position = current_joint_states.position[:]

                if "HeadYaw" in updated_joint_states.name:
                    head_yaw_idx = updated_joint_states.name.index("HeadYaw")
                    updated_joint_states.position[head_yaw_idx] = smoothed_yaw

                if "HeadPitch" in updated_joint_states.name:
                    head_pitch_idx = updated_joint_states.name.index("HeadPitch")
                    updated_joint_states.position[head_pitch_idx] = smoothed_pitch

                pub.publish(updated_joint_states)

                previous_yaw_angle = smoothed_yaw
                previous_pitch_angle = smoothed_pitch

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to lookup transform between 'Head' and 'l_gripper'")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
