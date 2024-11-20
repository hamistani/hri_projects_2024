#!/usr/bin/python3

import rospy
#from std_msgs.msg import String
from week0.msg import person


def main():
    rospy.set_param('/use_sim_time', False)

    rospy.init_node('talker', anonymous = True)
  # chatter_pub = rospy.Publisher('chatter', String, queue_size=1000)
    chatter_pub = rospy.Publisher('chatter', person, queue_size=1000)
    rate = rospy.Rate(10)

    count = 0
    while not rospy.is_shutdown():
        # msg_str = "hello world %s" % count
        # rospy.loginfo(msg_str)
        person_one = person(x = 1.0, y = 2.0, z = 3.0)

        rospy.loginfo(f"Publishing: x = {person_one.x}, y = {person_one.y}, z = {person_one.z}")
        chatter_pub.publish(person_one)

        # msg = String()
        # chatter_pub.publish(msg)

        rate.sleep()
        count += 1

if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass