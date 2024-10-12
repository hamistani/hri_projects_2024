#!/usr/bin/python3

import rospy
#from std_msgs.msg import String
from week0.msg import person

def chatter_callback(msg):
    #rospy.loginfo("I heard: [%s]", msg.data)
    rospy.loginfo(f"I heard: x = {msg.x}, y = {msg.y}, z = {msg.z}")

def main():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('chatter', person, chatter_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

