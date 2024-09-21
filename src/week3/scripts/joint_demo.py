#!/usr/bin/python3
# license removed for brevity
import rospy
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

def move_joints(pub, names, position, delay = 1):
    js = JointState()
    js.header = Header()
    js.header.stamp = rospy.get_rostime()
    js.name = names
    js.position = position

    pub.publish(js)
    rospy.loginfo(f"Moving joints: {names} to positions {position}")

    rospy.sleep(delay)

def nod_head(pub):
    rospy.loginfo("Robot is nodding its head")
    move_joints(pub, ["HeadPitch"], [math.radians(-20)], delay = 0.5)
    move_joints(pub, ["HeadPitch"], [math.radians(20)], delay = 0.5)
    move_joints(pub, ["HeadPitch"], [math.radians(0)], delay = 0.5)

def wave_hi(pub):
    rospy.loginfo("Robot is waving hi")
    for i in range(3):
        move_joints(pub, ["LShoulderPitch", "LElbowYaw"], [math.radians(45), math.radians(90)], delay = 0.5)
        move_joints(pub, ["LShoulderPitch", "LElbowYaw"], [math.radians(60), math.radians(90)], delay = 0.5)
    move_joints(pub, ["LShoulderPitch", "LElbowYaw"], [math.radians(0), math.radians(0)], delay = 0.5)


def shake_head(pub):
    rospy.loginfo("Robot shaking its Head")
    move_joints(pub, ["HeadYaw"], [math.radians(-30)], delay = 0.5)
    move_joints(pub, ["HeadYaw"], [math.radians(30)], delay = 0.5)
    move_joints(pub, ["HeadYaw"], [math.radians(0)], delay = 0.5)

def interpolated_movement(pub, joint, start, end, steps = 10, delay = 0.1):
    step_size = (end - start) / steps
    for i in range(steps):
        position = start + step_size * i
        move_joints(pub, [joint], [position], delay)



def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    #rate = rospy.Rate(1) # 10hz

    # set initial angle
    #angle = 0


    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        # js = JointState()
        
        # # header info
        # js.header.stamp = rospy.get_rostime()
        # js.header.frame_id="Torso"


        # # put in some joints that we'll edit
        # js.name.append("HeadYaw")
        # js.name.append("HeadPitch")

        # js.position.append(math.radians(angle))
        # js.position.append(0)

        # #comment this out once it gets noisy
        # rospy.loginfo(js)
        
        # pub.publish(js)
        # angle = angle + 1
        # rate.sleep()

        rospy.sleep(2)
        nod_head(pub)

        rospy.sleep(1)
        wave_hi(pub)
        rospy.sleep(1)
        shake_head(pub)

        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass