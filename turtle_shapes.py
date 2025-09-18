#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math

class TurtleShapes:
    def __init__(self):
        rospy.init_node('turtle_shapes_publisher', anonymous=True)
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def move_circle(self, speed=1.0, radius=1.0, duration=6.28):
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = speed / radius
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration:
            self.pub.publish(twist)
            self.rate.sleep()
        self.stop()

    def move_triangle(self, speed=1.0, side_length=2.0):
        twist = Twist()
        for _ in range(3):
            # Move forward
            twist.linear.x = speed
            twist.angular.z = 0
            start_time = rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec() - start_time < side_length / speed:
                self.pub.publish(twist)
                self.rate.sleep()
            self.stop()
            rospy.sleep(0.5)
            # Turn 120 degrees
            twist.linear.x = 0
            twist.angular.z = math.radians(120) / 1.0  # 1 second for 120 deg
            start_time = rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec() - start_time < 1.0:
                self.pub.publish(twist)
                self.rate.sleep()
            self.stop()
            rospy.sleep(0.5)

    def stop(self):
        twist = Twist()
        self.pub.publish(twist)

if __name__ == '__main__':
    shapes = TurtleShapes()
    rospy.sleep(2)
    shapes.move_circle()
    rospy.sleep(2)
    shapes.move_triangle()
    shapes.stop()
