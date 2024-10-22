#!/usr/bin/python3
import rospy
import math
import tf2_ros
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from FunctionCallback import FunctionCallback
from move import Move


publisher = None
subscriber = None
js_sub = None

js_msg = None

def callback_function(msg):
    global js_msg
    js_msg = msg  # Store the incoming JointState message in a global variable
    rospy.loginfo("Joint state updated!")

def main():
    # global publisher, subscriber, js_sub

    # initialize the node
    rospy.init_node('tf2_loo_at_hand2')

    # Create a buffer for storing transformations
    tfBuffer = tf2_ros.Buffer()

    # Make a subscriber for the function call
    listener = tf2_ros.TransformListener(tfBuffer)

    # Make a susbcriber to joint states
    js_sub = rospy.Subscriber('/joint_states', JointState, callback_function)


    # make a publisher to /joint_states
    publisher = rospy.Publisher('/updated_joint_states', JointState, queue_size = 10)

    function_callback = FunctionCallback()
    mover = Move()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            # Looks up the transformation between the head and the left gripper
            trans = tfBuffer.lookup_transform('HeadYaw', 'LHand', rospy.Time())

            # Calculates the yaw and pitch angles based on the transform
            yaw_angle = math.atan2(trans.transform.translation.y, trans.transform.translation.x)
            pitch_angle = math.atan2(trans.transform.translation.y, trans.transform.translation.x)

            # Creates a new JointState message to update the head's joints
            updated_joint_states = JointState()
            updated_joint_states.header = Header()
            updated_joint_states.header.stamp = rospy.get_rostime()
            updated_joint_states.name = ["HeadYaw", "HeadPitch"]
            updated_joint_states.position = [yaw_angle, pitch_angle]


            # Publish the updated joint States
            publisher.publish(updated_joint_states)
            rospy.loginfo(f"Moving head to yaw {yaw_angle}, pitch: {pitch_angle}")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
            rospy.logwarn(f"Transform lookup failed: {err}")
            rate.sleep()
            continue

        rate.sleep()



    rospy.spin() # Keeps node running until manaully stopped 

if __name__ == '__main__':
    # rospy.init('my_gesture_node')

    try:
        main()

    except rospy.ROSInterruptException:
        pass