import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import math
import time

class MovementController(Node):
    def __init__(self):
        super().__init__('movement_controller')

        # Dynamically locate the package directory
        package_share_directory = get_package_share_directory('movement_controller')

        # Construct the path to the YAML file
        yaml_file_path = os.path.join(package_share_directory, 'resource', 'test_movement.yaml')

        self.get_logger().info(f"YAML file path: {yaml_file_path}")
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Robot state variables
        self.current_x = 0.0  # Robot's X position
        self.current_y = 0.0  # Robot's Y position
        self.current_yaw = 0.0  # Robot's orientation (yaw)

        self.get_logger().info("Movement Controller Node Started")
        self.load_yaml_and_execute(yaml_file_path)

    def load_yaml_and_execute(self, yaml_file_path):
        try:
            with open(yaml_file_path, 'r') as file:
                movements = yaml.safe_load(file)['unit_moves']
                self.get_logger().info(f"Loaded movements from YAML: {movements}")
        except Exception as e:
            self.get_logger().error(f"Error loading YAML file: {e}")
            return

        # Execute each movement in sequence
        for move in movements:
            self.execute_movement(move)

    def execute_movement(self, move):
        twist = Twist()
        duration = 0.0

        # Determine motion type: rotate or straight
        if move['moving_type'] == 'rotate':
            angular_speed = math.radians(move['speed'])
            angle = math.radians(move['target']['angle'])
            duration = angle / angular_speed
            twist.angular.z = angular_speed if move['direction'] == 'right' else -angular_speed
        elif move['moving_type'] == 'straight':
            linear_speed = move['speed']
            distance = move['distance']
            duration = distance / linear_speed
            twist.linear.x = linear_speed if move['direction'] == 'forward' else -linear_speed

        # Publish the command and update robot's pose
        self.publish_command(twist, duration)

    def publish_command(self, twist, duration):
        start_time = time.time()
        while time.time() - start_time < duration:
            # Publish the velocity command
            self.cmd_vel_publisher.publish(twist)

            # Simulate the robot's movement in RViz
            self.update_robot_pose(twist, 0.1)  # Update every 0.1 seconds
            time.sleep(0.1)

        # Stop the robot after the movement
        self.cmd_vel_publisher.publish(Twist())

    def update_robot_pose(self, twist, delta_time):
        # Update robot position and orientation based on velocity
        delta_x = twist.linear.x * math.cos(self.current_yaw) * delta_time
        delta_y = twist.linear.x * math.sin(self.current_yaw) * delta_time
        delta_yaw = twist.angular.z * delta_time

        self.current_x += delta_x
        self.current_y += delta_y
        self.current_yaw += delta_yaw

        # Normalize yaw to stay within [-pi, pi]
        self.current_yaw = (self.current_yaw + math.pi) % (2 * math.pi) - math.pi

        # Publish the updated transform for the robot's base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"  # Fixed frame for visualization
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.current_x
        t.transform.translation.y = self.current_y
        t.transform.translation.z = 0.0
        q = self.yaw_to_quaternion(self.current_yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def yaw_to_quaternion(self, yaw):
        # Convert yaw to quaternion
        return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


def main(args=None):
    rclpy.init(args=args)
    node = MovementController()
    rclpy.spin(node)
    rclpy.shutdown()
