#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        
        # Publishers (steering & throttle)
        self.steering_pub = self.create_publisher(Float32, '/autodrive/roboracer_1/steering_command', 10)
        self.throttle_pub = self.create_publisher(Float32, '/autodrive/roboracer_1/throttle_command', 10)
        
        # Subscriber (LIDAR)
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/autodrive/roboracer_1/lidar',
            self.lidar_callback,
            10
        )

        # Parameters
        self.desired_distance = 1.0  # meters from wall
        self.max_speed = 1.5         # max throttle
        self.kp = 1.0                # proportional gain for wall following

        self.get_logger().info("Wall follower node started")

    def lidar_callback(self, msg: LaserScan):
        # Basic wall-following: look at left side
        left_angle_index = len(msg.ranges) * 3 // 4  # 270 degrees if 0 = front
        left_distance = msg.ranges[left_angle_index]

        # Compute steering based on distance error
        error = self.desired_distance - left_distance
        steering = self.kp * error

        # Clamp steering
        steering = max(min(steering, 1.0), -1.0)

        # Constant forward throttle
        throttle = self.max_speed

        # Publish commands
        self.steering_pub.publish(Float32(data=steering))
        self.throttle_pub.publish(Float32(data=throttle))

        # Optional: log values
        self.get_logger().info(f"LIDAR left: {left_distance:.2f} | Error: {error:.2f} | Steering: {steering:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
