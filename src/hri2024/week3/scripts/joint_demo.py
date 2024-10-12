#!/usr/bin/python3
# license removed for brevity
import rospy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
# import time

#joint names
joint_names = ["HeadYaw", "HeadPitch", "gaze_joint", "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll", "LLeg_effector_fixedjoint", "RHipYawPitch", 
"RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll", "RLeg_effector_fixedjoint", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand", 
"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand", "LFoot/Bumper/Right_sensor_fixedjoint", "RFoot/FSR/RearLeft_sensor_fixedjoint", "CameraBottom_optical_frame_fixedjoint",
"RFoot/FSR/RearRight_sensor_fixedjoint", "LFoot/FSR/FrontRight_sensor_fixedjoint", "LHand/Touch/Back_sensor_fixedjoint", "LHand/Touch/Left_sensor_fixedjoint", "RFoot/Bumper/Right_sensor_fixedjoint",
"RFoot/FSR/FrontRight_sensor_fixedjoint", "MicroLeft_sensor_fixedjoint", "LFoot/FSR/RearRight_sensor_fixedjoint", "MicroRight_sensor_fixedjoint", "Accelerometer_sensor_fixedjoint", "Head/Touch/Front_sensor_fixedjoint", 
"RFoot/Bumper/Left_sensor_fixedjoint", "Sonar/Right_sensor_fixedjoint", "RHand/Touch/Right_sensor_fixedjoint", "RHand/Touch/Back_sensor_fixedjoint", "RFoot/FSR/FrontLeft_sensor_fixedjoint", "Sonar/Left_sensor_fixedjoint",
"LFoot/FSR/FrontLeft_sensor_fixedjoint", "LFoot/FSR/RearLeft_sensor_fixedjoint", "RHand/Touch/Left_sensor_fixedjoint", "CameraBottom_sensor_fixedjoint", "CameraTop_sensor_fixedjoint", "InfraredL_sensor_fixedjoint",
"Gyrometer_sensor_fixedjoint", "MicroFront_sensor_fixedjoint", "Head/Touch/Rear_sensor_fixedjoint", "Head/Touch/Middle_sensor_fixedjoint", "InfraredR_sensor_fixedjoint", "ChestBoard/Button_sensor_fixedjoint", "LFoot/Bumper/Left_sensor_fixedjoint",
"MicroRear_sensor_fixedjoint", "LHand/Touch/Right_sensor_fixedjoint", "RFinger23", "RFinger13", "RFinger12", "LFinger21", "LFinger13", "LFinger11", "RFinger22", "LFinger22", "RFinger21", "LFinger12", "RFinger11", "LFinger23", "LThumb1", "RThumb1",
"RThumb2", "LThumb2"]

current_joint_states = JointState()

def joint_state_callback(msg):
    global current_joint_states
    current_joint_states = msg

def move_head(pub):
    global current_joint_states

    updated_joint_states = JointState()
    updated_joint_states.header = Header()
    updated_joint_states.header.stamp = rospy.get_rostime()
    updated_joint_states.name = current_joint_states.name[:]
    updated_joint_states.position = current_joint_states.position[:]

    joint_positions = current_joint_staes.position

    head_yaw_idx = updated_joint_states.name.index("HeadYaw")
    head_yaw_idx = updated_joint_states.name.index("HeadYaw")

    joint_positions[head_yaw_idx] = math.radians(30)
    joint_positions[head_pitch_idx] = math.radians(-20)

    updated_joint_states.position = joint_positions

    pub.publish(updated_joint_states)
    rospy.loginfo("Moving head joints")

    def main():
        ropy.init_node('head_mover')

        pub = rospy.Publisher('/joint_states', JointState, queue_size = 10)

        rospy.Subscriber('/joint_state_input', JointState, joint_state_callback)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if current_joint_states:
                move_head(pub)
            rate.sleep()

    if __name__ == '__main__':
        try:
            main()
        except rospy.ROSInterruptException:
            pass


# def move_joints(pub, names, position, delay = 1):
#     js = JointState()
#     js.header = Header()
#     js.header.stamp = rospy.get_rostime()
#     js.name = names
#     js.position = position

#     pub.publish(js)
#     rospy.loginfo(f"Moving joints: {names} to positions {position}")

#     rospy.sleep(delay)

# def nod_head(pub):
#     rospy.loginfo("Robot is nodding its head")
#     move_joints(pub, ["HeadPitch"], [math.radians(-20)], delay = 0.5)
#     move_joints(pub, ["HeadPitch"], [math.radians(20)], delay = 0.5)
#     move_joints(pub, ["HeadPitch"], [math.radians(0)], delay = 0.5)

# def wave_hi(pub):
#     rospy.loginfo("Robot is waving hi")
#     for i in range(3):
#         move_joints(pub, ["LShoulderPitch", "LElbowYaw"], [math.radians(45), math.radians(90)], delay = 0.5)
#         move_joints(pub, ["LShoulderPitch", "LElbowYaw"], [math.radians(60), math.radians(90)], delay = 0.5)
#     move_joints(pub, ["LShoulderPitch", "LElbowYaw"], [math.radians(0), math.radians(0)], delay = 0.5)


# def shake_head(pub):
#     rospy.loginfo("Robot shaking its Head")
#     move_joints(pub, ["HeadYaw"], [math.radians(-30)], delay = 0.5)
#     move_joints(pub, ["HeadYaw"], [math.radians(30)], delay = 0.5)
#     move_joints(pub, ["HeadYaw"], [math.radians(0)], delay = 0.5)

# def interpolated_movement(pub, joint, start, end, steps = 10, delay = 0.1):
#     step_size = (end - start) / steps
#     for i in range(steps):
#         position = start + step_size * i
#         move_joints(pub, [joint], [position], delay)



# def talker():
#     pub = rospy.Publisher('joint_states', JointState, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     #rate = rospy.Rate(1) # 10hz

#     # set initial angle
#     #angle = 0


#     while not rospy.is_shutdown():
#         #hello_str = "hello world %s" % rospy.get_time()
#         # js = JointState()
        
#         # # header info
#         # js.header.stamp = rospy.get_rostime()
#         # js.header.frame_id="Torso"


#         # # put in some joints that we'll edit
#         # js.name.append("HeadYaw")
#         # js.name.append("HeadPitch")

#         # js.position.append(math.radians(angle))
#         # js.position.append(0)

#         # #comment this out once it gets noisy
#         # rospy.loginfo(js)
        
#         # pub.publish(js)
#         # angle = angle + 1
#         # rate.sleep()

#         rospy.sleep(2)
#         nod_head(pub)

#         rospy.sleep(1)
#         wave_hi(pub)
#         rospy.sleep(1)
#         shake_head(pub)

        

# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass