#!/usr/bin/python3
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import time

# Publisher names
pub_arm = rospy.Publisher('/joint_states_out', JointState, queue_size=10)
pub_head = rospy.Publisher('/joint_states_out', JointState, queue_size=10)

def gesture_callback(msg):
    try:
        recognized_speech = msg.data
        rospy.loginfo(f"Recognized Speech: {recognized_speech}")

        # Check for keywords such as hello, hi, or bye
        if 'hi' in recognized_speech.lower() or 'hello' in recognized_speech.lower() or 'bye' in recognized_speech.lower():
            rospy.loginfo("Waving")
            wave_robot()

        elif 'yes' in recognized_speech.lower():
            rospy.loginfo("Nodding")
            nod_robot()

        elif 'no' in recognized_speech.lower():
            rospy.loginfo("Shaking Head")
            shake_head()

    except Exception as e:
        rospy.logerr(f"Error in gesture_callback: {e}")

# Robot waves hi, hello, goodbye
def wave_robot():
    rospy.loginfo("Executing Wave Gesture...")
    arm_joint_states = JointState()
    arm_joint_states.header.stamp = rospy.get_rostime()

    # Joint names
    arm_joint_states.name = ["LElbowRoll", "LElbowYaw", "LShoulderPitch", "LShoulderRoll"]

    # Sets the initial arm position 
    arm_joint_states.position = [1.0, 0.5, -1.0, 1.0] 
    pub_arm.publish(arm_joint_states)
    rospy.sleep(1)

    base_time = time.time()

    while not rospy.is_shutdown():
        current_time = time.time() - base_time
        # Adjusts speed
        arm_joint_states.position[0] = .2 * math.sin(current_time * 2)
        arm_joint_states.header.stamp = rospy.get_rostime()
        pub_arm.publish(arm_joint_states) 
        
        rospy.sleep(0.1)

        # Operate for 2 seconds
        if current_time > 2:  
            break
    
    # Return the arm to neutral position
    arm_joint_states.position = [0.0, 0.0, 0.0, 0.0]
    pub_arm.publish(arm_joint_states)
    rospy.loginfo("Wave gesture completed.")


# Robot nods
def nod_robot():
    rospy.loginfo("Executing Continuous Head Nod...")
    head_joint_states = JointState()
    head_joint_states.header.stamp = rospy.get_rostime()
    head_joint_states.name = ["HeadYaw", "HeadPitch"]

    base_time = time.time()

    while not rospy.is_shutdown():
        current_time = time.time() - base_time
        head_joint_states.position = [0, 0.5 * math.sin(current_time)] 
        head_joint_states.header.stamp = rospy.get_rostime()
        pub_head.publish(head_joint_states)

        rospy.sleep(0.1) 

        # Run for 5 seconds before stopping
        if current_time > 5:
            break

    head_joint_states.position = [0, 0]
    pub_head.publish(head_joint_states)
    rospy.loginfo("Head nod gesture completed.")

# Robot shakes head (No)
def shake_head():
    rospy.loginfo("Executing Continuous Head Shake...")
    head_joint_states = JointState()
    head_joint_states.header.stamp = rospy.get_rostime()
    head_joint_states.name = ["HeadYaw", "HeadPitch"]

    base_time = time.time()

    while not rospy.is_shutdown():
        current_time = time.time() - base_time
        head_joint_states.position = [math.sin(current_time) * 1.0, 0] 
        head_joint_states.header.stamp = rospy.get_rostime()
        pub_head.publish(head_joint_states)

        rospy.sleep(0.1) 

        # Run for 5 seconds before stopping
        if current_time > 5:
            break

    head_joint_states.position = [0, 0]
    pub_head.publish(head_joint_states)
    rospy.loginfo("Head shake gesture completed.")

def main():
    rospy.init_node('move_robot')
    rospy.loginfo("move_robot node started. Listening for speech...")

    rospy.Subscriber('speech_recognition/final_result', String, gesture_callback)

    rospy.loginfo("Subscriber set up successfully.") 

    rospy.spin() 

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
