#!/usr/bin/python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

SAFE_DISTANCE = 0.5  

class MoveUntilObstacle:
    def __init__(self):
        #Publisher for movement commands
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        #Subscriber for laser scan data
        self.sub = rospy.Subscriber("/base_scan", LaserScan, self.scan_callback)
        
        #Determines if the robot should move
        self.should_move = True

    def scan_callback(self, data):
        #Focuses on the front range of scan data to check for obstacles
        front_ranges = data.ranges[len(data.ranges) // 2 - 10 : len(data.ranges) // 2 + 10]
        closest_front_distance = min(front_ranges)

        rospy.loginfo(f"Closest distance to obstacle in front: {closest_front_distance:.2f} meters")

        #Updates movement state based on closest front obstacle
        if closest_front_distance < SAFE_DISTANCE:
            if self.should_move:
                rospy.loginfo("Obstacle detected! Stopping.")
            self.should_move = False
        else:
            if not self.should_move:
                rospy.loginfo("Path clear. Moving forward.")
            self.should_move = True

    def move(self):
        #Creates and publishes movement commands based on the current movement state
        move_cmd = Twist()
        
        if self.should_move:
            move_cmd.linear.x = 0.2  #Move forward
            rospy.loginfo("Moving forward.")
        else:
            move_cmd.linear.x = 0.0  #Stop
            rospy.loginfo("Stopping.")
        
        self.pub.publish(move_cmd)

if __name__ == '__main__':
    rospy.init_node('move_until_obstacle')
    node = MoveUntilObstacle()
    
    #Set a loop rate for regular checks
    rate = rospy.Rate(10)  # 10 Hz

    #Main loop
    while not rospy.is_shutdown():
        node.move()
        rate.sleep()
