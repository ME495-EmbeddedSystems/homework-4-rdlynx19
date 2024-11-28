"""
The Explore Node to autonomously explore the environment using random points.

Publishers
----------
+ /goal_pose (geometry_msgs/PoseStamped) - The goal pose of the robot

Subscribers
-----------
+ /map (nav_msgs/OccupancyGrid) - The current map of the environment

"""
import math
import random

from geometry_msgs.msg import PoseStamped, TransformStamped

from nav_msgs.msg import OccupancyGrid

import rclpy
import rclpy.time
from rclpy.node import Node

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def vec_to_pose(transform: TransformStamped, pose: PoseStamped):
    """
    Convert TransformStamped to PoseStamped.

    :param transform: transform stamped from map to base_link
    :param pose: pose stamped indicating the current robot pose
    """
    pose.pose.position.x = transform.transform.translation.x
    pose.pose.position.y = transform.transform.translation.y
    pose.pose.position.z = transform.transform.translation.z
    pose.pose.orientation = transform.transform.rotation
    pose.header.frame_id = transform.header.frame_id
    return pose


def calculate_euclidean_distance(robot_pose: PoseStamped,
                                 goal_pose: PoseStamped):
    """
    Calculate the planar distance between two points.

    :param robot_pose: the current pose of the robot
    :param goal_pose: the current goal pose
    """
    dist = math.sqrt(
        (robot_pose.pose.position.x - goal_pose.pose.position.x)**2 +
        (robot_pose.pose.position.y - goal_pose.pose.position.y)**2
    )

    return dist


class Explore(Node):
    """Node to explore the environment."""

    def __init__(self):
        """Initialise member variables of the class."""
        super().__init__('explore')
        # Map Subscriber
        self.map_sub = self.create_subscription(OccupancyGrid, 'map',
                                                self.map_callback, 10)
        # Goal Publisher
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # Timer to lookup map->base_link transform
        self.tf_pub = self.create_timer(1, self.pose_tmr)
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)

        # Helper variables to store the robot and goal poses
        self.robot_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.changing_goal_pose = PoseStamped()
        self.dist_tracker = []

    def pose_tmr(self):
        """Get the robot's pose wrt map."""
        try:
            map_baselink_lookup = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
                )
            empty_pose = PoseStamped()
            self.robot_pose = vec_to_pose(map_baselink_lookup, empty_pose)
        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().debug(f'Lookup exception: {e}')
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().debug(f'Connectivity exception: {e}')
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().debug(f'Extrapolation exception: {e}')

    def map_callback(self, map_msg):
        """
        Get the current map data.

        :param map_msg: the current map information
        :type map_msg: nav_msgs.msg.OccupancyGrid
        """
        occ_map = map_msg
        i = random.randint(1, len(occ_map.data)-2)
        if occ_map.data[i] == -1 or occ_map.data[i] == 0:
            row = i // occ_map.info.width
            col = i % occ_map.info.width
            self.changing_goal_pose.pose.position.x = (
                        occ_map.info.origin.position.x +
                        occ_map.info.resolution * col
                        )
            self.changing_goal_pose.pose.position.y = (
                        occ_map.info.origin.position.y +
                        occ_map.info.resolution * row
                        )
            if (calculate_euclidean_distance(self.robot_pose,
                                             self.changing_goal_pose) > 15.0):
                self.get_logger().info('Finding New Goal Pose...')
                # Skip the current goal pose if it is too far
                return
            else:
                # Send a new valid goal pose to be executed
                self.get_logger().info('Checking for New Goal Pose...')
                self.send_goal_pose()

    def send_goal_pose(self):
        """Publish a goal pose command to the robot."""
        current_dist_to_goal = calculate_euclidean_distance(
            self.robot_pose, self.goal_pose)
        self.dist_tracker.append(current_dist_to_goal)
        # Check if the robot is stuck or has stopped
        if (len(self.dist_tracker) > 1):
            if (-0.1 < self.dist_tracker[-2] - current_dist_to_goal < 0.1):
                current_dist_to_goal = 0.0
        if (current_dist_to_goal < 0.5):
            self.goal_pose.pose.position.x = (
                self.changing_goal_pose.pose.position.x)
            self.goal_pose.pose.position.y = (
                self.changing_goal_pose.pose.position.y)
            self.goal_pose.header.frame_id = 'map'
            self.get_logger().info(f'Goal Pose: {{x: {
                self.goal_pose.pose.position.x}, y: {
                    self.goal_pose.pose.position.y}}}')
            self.goal_pub.publish(self.goal_pose)


def main(args=None):
    """Spin the explore node."""
    rclpy.init(args=args)
    node = Explore()
    rclpy.spin(node)
    rclpy.shutdown()
