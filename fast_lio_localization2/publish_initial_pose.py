#!/usr/bin/env python3

import sys
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
import tf_transformations


class PublishInitialPose(Node):
    def __init__(self):
        super().__init__("publish_initial_pose")
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    def publish_pose(self, x, y, z, roll, pitch, yaw):
        quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        initial_pose = PoseWithCovarianceStamped()
        
        # ROS 2 requires keyword arguments for message instantiation
        p = Point(x=x, y=y, z=z)
        q = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        
        initial_pose.pose.pose = Pose(position=p, orientation=q)
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = "map"
        
        self.pub_pose.publish(initial_pose)

        self.get_logger().info(f"Initial Pose: x={x} y={y} z={z} yaw={yaw} pitch={pitch} roll={roll}")


def main(args=None):
    rclpy.init(args=args)
    node = PublishInitialPose()

    # Safely strip ROS 2 arguments out so argparse doesn't crash
    clean_args = rclpy.utilities.remove_ros_args(sys.argv)

    parser = argparse.ArgumentParser()
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("z", type=float)
    parser.add_argument("yaw", type=float)
    parser.add_argument("pitch", type=float)
    parser.add_argument("roll", type=float)
    
    parsed_args = parser.parse_args(clean_args[1:])

    node.publish_pose(
        parsed_args.x, parsed_args.y, parsed_args.z, 
        parsed_args.roll, parsed_args.pitch, parsed_args.yaw
    )
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()