#!/usr/bin/python3
import rospy
import math
from geometry_msgs.msg import Twist 
from move_straight import MoveStraight 
from turn_odom import TurnOdom

class MoveSquare:
    def __init__(self):
        #Initialize ROS node
        rospy.init_node('move_square')

        #Initialize MoveStraight and TurnOdom objects to control movement
        self.straight = MoveStraight()
        self.turn = TurnOdom()

    def move_straight_for_distance(self, distance = 2):
        t = Twist()
        t.linear.x = 0.5 #Set forward speed
        time = 0.0
        max_time = distance / 0.5
        while not rospy.is_shutdown() and time < max_time:    
            self.straight.pub.publish(t) #Publish speed to make the robot move
            rospy.sleep(.01) #Move for specified time 
            time += 0.1
        
        #Stop the robot by setting linear speed to zero 
        t.linear.x = 0.0
        self.straight.pub.publish(t)
        rospy.sleep(1) #Short pause to ensure the robot stops completely 

    def turn_90_degrees(self):
        start_yaw = self.turn.get_yaw(self.turn.get_odom()) #Get initial orientation 
        t = Twist()
        t.angular.z = 0.5 #Set rotation speed
        self.turn.pub.publish(t) #Start turning 

        cumulative_angle = 0
        previous_yaw = start_yaw

        while not rospy.is_shutdown():
            cur_yaw = self.turn.get_yaw(self.turn.get_odom()) #Get current orientation
            delta_yaw = (cur_yaw - previous_yaw + math.pi) % (2 * math.pi) - math.pi
            cumulative_angle += delta_yaw
            previous_yaw = cur_yaw
            self.turn.pub.publish(t) #Start turning 


            if abs(cumulative_angle) >= math.radians(85.5):
                t.angular.z = 0.0
                self.turn.pub.publish(t)
                rospy.sleep(1)
                break

            rospy.sleep(.005) #Short pause to prevent excessive looping 

    def move_in_square(self):
        for _ in range(4): #Repeat for four sides of the square 
            self.move_straight_for_distance(distance = 2) #Move straight 
            self.turn_90_degrees() #Turn 90 degrees 

if __name__ == '__main__':
    move_square = MoveSquare() #Create MoveSquare instance 
    move_square.move_in_square() #Start moving in a square

