#!/usr/bin/env python3

import rospy
from people_msgs.msg import PositionMeasurementArray, PositionMeasurement
from people_msgs.msg import People, Person
import math

class GroupDetector:
    def __init__(self):
        rospy.init_node('group_detector', anonymous=True)
        self.people_sub = rospy.Subscriber('/leg_tracker_measurements', PositionMeasurementArray, self.people_callback)
        self.group_pub = rospy.Publisher('/robot_0/detected_groups', People, queue_size=10)
        rospy.loginfo("Group Detector Node Initialized")

    def people_callback(self, msg):
        rospy.loginfo(f"Callback triggered with {len(msg.people)} people")
        
        # Convert PositionMeasurementArray to People
        people = self.convert_to_people(msg)
        groups = self.identify_groups(people)
        updated_people = []

        for group_name, group_members in groups.items():
            rospy.loginfo(f"Group {group_name} has {len(group_members)} members")
            for person in group_members:
                rospy.loginfo(f"Person {person.name} in {group_name}")
                person.name = f"{group_name}_{person.name}"
                updated_people.append(person)

        if updated_people:
            detected_groups = People(header=msg.header, people=updated_people)
            self.group_pub.publish(detected_groups)
            rospy.loginfo(f"Published {len(updated_people)} people to /robot_0/detected_groups")
        else:
            rospy.loginfo("No groups detected.")

    def convert_to_people(self, msg):
        """Convert PositionMeasurementArray to People."""
        people = []
        for measurement in msg.people:
            person = Person(
                name=measurement.object_id, 
                position=measurement.pos, 
                reliability=measurement.reliability
            )
            people.append(person)
        return people

    def identify_groups(self, people):
        groups = {"circle_1": [], "line_1": []}
        if len(people) < 2:
            rospy.loginfo("Not enough people to form a group")
            return groups

        # Detect circle groups
        center_x, center_y, count = 0, 0, 0
        for person in people:
            center_x += person.position.x
            center_y += person.position.y
            count += 1
        center_x /= count
        center_y /= count

        rospy.loginfo(f"Calculated circle center: ({center_x}, {center_y})")

        radius_threshold = 1.5
        circle_members = [
            person for person in people
            if math.sqrt((person.position.x - center_x) ** 2 + (person.position.y - center_y) ** 2) <= radius_threshold
        ]

        rospy.loginfo(f"Detected {len(circle_members)} members in circle group")

        if len(circle_members) > 2:
            groups["circle_1"] = circle_members

        # Detect line groups
        line_members = []
        for i, person1 in enumerate(people):
            for j, person2 in enumerate(people):
                if i >= j:
                    continue
                dist = self.euclidean_distance(person1.position, person2.position)
                rospy.loginfo(f"Distance between {person1.name} and {person2.name}: {dist}")
                if 1.0 <= dist <= 1.5:
                    line_members.extend([person1, person2])

        rospy.loginfo(f"Detected {len(line_members)} members in line group")

        if len(line_members) > 2:
            groups["line_1"] = line_members

        return groups

    @staticmethod
    def euclidean_distance(pos1, pos2):
        return ((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2) ** 0.5


if __name__ == "__main__":
    node = GroupDetector()
    rospy.spin()
