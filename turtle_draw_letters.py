#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# Replace these with your initials if you want different letters
def draw_M(pub, rate):
    # Draws a capital 'M' with the turtle
    cmds = [
        # Left vertical
        (1.5, 0, 1.5),
        (0, math.radians(-135), 0.7),
        # Left diagonal up
        (1.0, 0, 1.0),
        (0, math.radians(90), 0.7),
        # Middle down
        (1.0, 0, 1.0),
        (0, math.radians(-135), 0.7),
        # Right vertical
        (1.5, 0, 1.5)
    ]
    for lin, ang, dur in cmds:
        twist = Twist()
        twist.linear.x = lin
        twist.angular.z = ang
        start = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start < dur:
            pub.publish(twist)
            rate.sleep()
        pub.publish(Twist())
        rospy.sleep(0.5)

def draw_B(pub, rate):
    # Draws a capital 'B' with the turtle
    cmds = [
        # Vertical line
        (1.5, 0, 1.5),
        (0, math.radians(-90), 0.7),
        # Top half-circle
        (0.5, math.radians(90), 1.0),
        (0, math.radians(-90), 0.7),
        # Middle
        (0.5, 0, 0.5),
        (0, math.radians(-90), 0.7),
        # Bottom half-circle
        (0.5, math.radians(90), 1.0)
    ]
    for lin, ang, dur in cmds:
        twist = Twist()
        twist.linear.x = lin
        twist.angular.z = ang
        start = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start < dur:
            pub.publish(twist)
            rate.sleep()
        pub.publish(Twist())
        rospy.sleep(0.5)


class DrawLetters:
    def __init__(self):
        rospy.init_node('turtle_draw_letters', anonymous=True)
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.pose = None
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        rospy.sleep(2)

    def pose_callback(self, msg):
        self.pose = msg

    def run(self):
        draw_M(self.pub, self.rate)
        rospy.sleep(1)
        draw_B(self.pub, self.rate)
        self.pub.publish(Twist())

if __name__ == '__main__':
    node = DrawLetters()
    node.run()
