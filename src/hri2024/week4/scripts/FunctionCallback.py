#!/usr/bin/python3
import rospy
import math
import tf2_ros
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64 
from move import Move
import geometry_msgs.msg



publisher = None
subscriber = None
js_sub = None

js_msg = None

class FunctionCallback:

    def __init__(self):
        self.counter = 0
        self.pub = rospy.Publisher("/function_callback", Int64, queue_size = 10)
        self.function_subscriber = rospy.Subscriber("/function", Int64, self.function_callback)
        self.js_subscriber = rospy.Subscriber("/joint_states", JointState, self.js_callback)
        self.msg = Int64()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.joint_pub = rospy.Publisher("/updated_joint_states", JointState, queue_size = 10)

    # actually moves the robot
    def function_callback(self, msg):

        # look up head position wrt torso
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        try: 
            head_to_torso = tfBuffer.lookup_transform('torso', 'HeadYaw', rospy.Time())
            rospy.loginfo('Success, head to torso transform has been gotten')

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform lookup failed for head to torso")

            return


        # look up left handed position wrt torso
        try: 
            hand_to_torso = tfBuffer.lookup_transform('torso', 'l_gripper', rospy.Time())
            rospy.loginfo('Success, head to torso transform has been gotten')

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform lookup failed for hand to torso")

            return

        # get vector from head to left hand wrt torso
        head_to_hand_vector = geometry_msgs.msg.Vector3(
            hand_to_torso.transform.translation.x - head_to_torso.transform.translation.x,
            hand_to_torso.transform.translation.y - head_to_torso.transform.translation.y,
            hand_to_torso.transform.translation.z - head_to_torso.transform.translation.z
        )

        # transform that vector from torso frame to head frame
        try:
            hand_to_head_transform = tfBuffer.lookup_transform('HeadYaw', 'LHand', rospy.Time())
            rospy.loginfo("Success, vector to head frame")
        
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform lookup failed for hand to head")
            return

        # atan2 to get angle (yaw, pitch)
        yaw = math.atan2(head_to_hand_vector.y, head_to_hand_vector.x)
        pitch = math.atan2(head_to_hand_vector.z, head_to_hand_vector.x)

        # move
        Move.move(yaw, pitch)

        # Step 4: (1, 0, 0) in handframe transform straight to headframe atan2 to get yaw and pitch
        move_yaw = math.atan2(hand_to_head_transform.transform.translation.y, hand_to_head_transform.transform.translation.x)
        move_pitch = math.atan2(hand_to_head_transform.transform.translation.z, hand_to_head_transform.translation.x)

        rospy.loginfo(f"Move to yaw: {yaw}, pitch: {pitch}")

    def process_joint_state(self, msg):
        # Process the incoming joint state message, and return the updated message
        updated_joint_states = JointState()

        # Copy over the saved one
        updated_joint_states.header = msg.header
        updated_joint_states.name = msg.name
        updated_joint_states.position = list(msg.position)  # Ensure it's a mutable list

        # Example logic to update head yaw and pitch (modify as needed)
        if "HeadYaw" in updated_joint_states.name and "HeadPitch" in updated_joint_states.name:
            yaw_index = updated_joint_states.name.index("HeadYaw")
            pitch_index = updated_joint_states.name.index("HeadPitch")
            
            # Set yaw and pitch to some calculated values (you can replace this with actual logic)
            updated_joint_states.position[yaw_index] = 0.5  # Replace with actual yaw value
            updated_joint_states.position[pitch_index] = 0.3  # Replace with actual pitch value

        return updated_joint_states


    def js_callback(self, msg): 
        rospy.loginfo("Joint state callback invoked")
        updated_joint_states = self.process_joint_state(msg)
        if updated_joint_states:
            self.joint_pub.publish(updated_joint_states)
            rospy.loginfo("Joint state updated")

