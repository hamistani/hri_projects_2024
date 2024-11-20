#!/usr/bin/python3
import rospy
import math
import tf2_ros
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


publisher = None
subscriber = None
js_sub = None

js_msg = None

class Move:

    def move(yaw, pitch):
        global publisher, js_msg

        # make a new joint state msg
        updated_joint_states = JointState()

        # copy over the saved one
        if js_msg is not None:
            updated_joint_states.header = js_msg.header
            updated_joint_states.name = js_msg.name
            updated_joint_states.position = js_msg.position[:]
        else:
            rospy.logwarn("No joint state message available to copy from")
            return 

        # change headpitch and headyaw
        if "HeadYaw" in updated_joint_states.name and "HeadPitch" in updated_joint_states.name:
            # yaw_index = updated_joint_states.name.index("HeadYaw")
            # pitch_index = updated_joint_states.name.index("HeadPitch")
            smoothed_yaw = previous_yaw_angle + smoothing_factor * (yaw - previous_yaw_angle)
            smoothed_pitch = previous_pitch_angle + smoothing_factor * (pitch - previous_pitch_angle)


            updated_joint_states.position[yaw_index] = yaw
            updated_joint_states.position[pitch_index] = pitch

            rospy.loginfo(f"Updated head joints: yaw={yaw}, pitch={pitch}")
        
        else:
            rospy.logwarn("HeadYaw or HeadPitch not found in the joint states")
            return 

        # publish
        publisher.publish(updated_joint_states)
        rospy.loginfo("Published updated joint states.")

        pass