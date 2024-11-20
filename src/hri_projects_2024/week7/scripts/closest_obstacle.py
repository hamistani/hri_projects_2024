#!/usr/bin/python3
import rospy
from sensor_msgs.msg import LaserScan

#Define the callback function that processes incoming laser scan data
def base_scan_callback(data):
    #Check if the ranges array has data
    if data.ranges:
        #Find the minimum distance eeeein the ranges array (closest array)
        closest_distance = min(data.ranges)

        print("Recieved Laserscan data")
        print("Closest obstacle distance (meters): {:2f}".format(closest_distance))
        
        #Log the closest distance to teh console for debugging/observation
        rospy.loginfo("Closest obstacle is at distance: {:.2f} meters".format(closest_distance))
    else:
        #No data recieved, log a wanring message
        rospy.logwarn("No data recieved from base_scan")

def main():
    #Initialize a new ROS node 
    rospy.init_node('closest_obstacle_listener', anonymous = True)

    #Setup a subscription 
    rospy.Subscriber("/base_scan", LaserScan, base_scan_callback)

    #Keep the node active and waiting for callbacks until manually stopped 
    rospy.spin()

#Run the main function when this script is executed
if __name__ == '__main__':
        main()

