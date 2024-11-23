#!/usr/bin/python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

publisher = None
js_msg = None
previous_yaw_angle = 0.0
previous_pitch_angle = 0.0
smoothing_factor = 0.5

class Move:
    @staticmethod
    def move(yaw, pitch):
        global publisher, js_msg, previous_yaw_angle, previous_pitch_angle

        # Ensure a joint state message is available
        if js_msg is None:
            rospy.logwarn("No joint state message available to copy from")
            return

        # Create a new joint state message
        updated_joint_states = JointState()
        updated_joint_states.header = Header()
        updated_joint_states.header.stamp = rospy.Time.now()
        updated_joint_states.name = js_msg.name[:]
        updated_joint_states.position = js_msg.position[:]

        # Find indices for HeadYaw and HeadPitch
        try:
            yaw_index = updated_joint_states.name.index("HeadYaw")
            pitch_index = updated_joint_states.name.index("HeadPitch")
        except ValueError:
            rospy.logwarn("HeadYaw or HeadPitch not found in joint states")
            return

        # Apply smoothing to yaw and pitch
        smoothed_yaw = previous_yaw_angle + smoothing_factor * (yaw - previous_yaw_angle)
        smoothed_pitch = previous_pitch_angle + smoothing_factor * (pitch - previous_pitch_angle)

        # Update positions
        updated_joint_states.position[yaw_index] = smoothed_yaw
        updated_joint_states.position[pitch_index] = smoothed_pitch

        # Publish the updated joint states
        publisher.publish(updated_joint_states)
        rospy.loginfo(f"Published updated joint states: yaw={smoothed_yaw}, pitch={smoothed_pitch}")

        # Update previous angles
        previous_yaw_angle = smoothed_yaw
        previous_pitch_angle = smoothed_pitch


def joint_state_callback(msg):
    global js_msg
    rospy.loginfo("Received joint state message")
    js_msg = msg


def main():
    global publisher

    rospy.init_node('move_node')

    # Initialize publisher and subscriber
    publisher = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.Subscriber('joint_states_input', JointState, joint_state_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
