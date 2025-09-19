#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

def draw_M(pub, rate_hz=10):
    cmds = [
        # Left vertical
        (1.5, 0, 1.5),
        (0, -math.radians(135), 0.7),
        # Left diagonal up
        (1.0, 0, 1.0),
        (0, math.radians(90), 0.7),
        # Middle down
        (1.0, 0, 1.0),
        (0, -math.radians(135), 0.7),
        # Right vertical
        (1.5, 0, 1.5)
    ]
    for lin, ang, dur in cmds:
        twist = Twist()
        twist.linear.x = lin
        twist.angular.z = ang
        start = time.time()
        while time.time() - start < dur:
            pub.publish(twist)
            time.sleep(1.0 / rate_hz)
        pub.publish(Twist())
        time.sleep(0.5)

def draw_B(pub, rate_hz=10):
    cmds = [
        # Vertical line
        (1.5, 0, 1.5),
        (0, -math.radians(90), 0.7),
        # Top half-circle (approximate)
        (0.5, math.radians(90), 1.0),
        (0, -math.radians(90), 0.7),
        # Middle
        (0.5, 0, 0.5),
        (0, -math.radians(90), 0.7),
        # Bottom half-circle (approximate)
        (0.5, math.radians(90), 1.0)
    ]
    for lin, ang, dur in cmds:
        twist = Twist()
        twist.linear.x = lin
        twist.angular.z = ang
        start = time.time()
        while time.time() - start < dur:
            pub.publish(twist)
            time.sleep(1.0 / rate_hz)
        pub.publish(Twist())
        time.sleep(0.5)

class DrawLetters(Node):
    def __init__(self):
        super().__init__('turtle_draw_letters')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info("Turtle ready to draw letters...")

    def run(self):
        time.sleep(2)  # wait for turtlesim_node
        draw_M(self.pub)
        time.sleep(1)
        draw_B(self.pub)
        self.pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = DrawLetters()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
