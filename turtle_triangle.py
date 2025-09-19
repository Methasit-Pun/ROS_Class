import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class TurtleShapes(Node):
    def __init__(self):
        super().__init__('turtle_shapes_publisher')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer_period = 0.1
        self.twist = Twist()

    def move_circle(self, speed=1.0, radius=1.0, duration=6.28):
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = speed / radius
        start_time = time.time()
        while time.time() - start_time < duration:
            self.pub.publish(twist)
            time.sleep(0.1)
        self.stop()

    def move_triangle(self, speed=1.0, side_length=2.0):
        twist = Twist()
        for _ in range(3):
            # Move forward
            twist.linear.x = speed
            twist.angular.z = 0.0
            start_time = time.time()
            while time.time() - start_time < side_length / speed:
                self.pub.publish(twist)
                time.sleep(0.1)
            self.stop()
            time.sleep(0.5)
            # Turn 120 degrees
            twist.linear.x = 0.0
            twist.angular.z = math.radians(120)
            start_time = time.time()
            while time.time() - start_time < 1.0:
                self.pub.publish(twist)
                time.sleep(0.1)
            self.stop()
            time.sleep(0.5)

    def stop(self):
        twist = Twist()
        self.pub.publish(twist)

def main():
    rclpy.init()
    node = TurtleShapes()
    time.sleep(2)
    # node.move_circle()
    node.move_triangle()
    time.sleep(2)
    node.stop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
