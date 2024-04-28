import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
import numpy as np


class WaypointFollowerNode:

    def __init__(self):
        self.node = rclpy.create_node('waypoint_follower')
        self.action_client = ActionClient(self.node, FollowWaypoints, 'follow_waypoints')
        self.goal_handle = None

    def send_waypoints(self, waypoints):
        self.node.get_logger().info('Sending waypoints...')  # Mensaje de registro
        goal_msg = FollowWaypoints.Goal()
        for waypoint in waypoints:
            pose = PoseStamped()
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = waypoint[2]
            pose.pose.orientation.w = waypoint[3]
            goal_msg.poses.append(pose)

        self.action_client.wait_for_server()
        self.goal_handle = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        self.node.get_logger().info('Feedback received: {0}'.format(feedback_msg.feedback.waypoint_completed))  # Mensaje de registro

    def wait_for_completion(self):
        self.node.get_logger().info('Waiting for waypoint following to complete...')  # Mensaje de registro
        if self.goal_handle is not None:
            self.goal_handle.result()


def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollowerNode()

    # Define waypoints
    waypoints = [
        [1.0, 2.0, 0.0, 1.0],  # x, y, z, w (orientation)
        [3.0, 4.0, 0.0, 1.0],
        [5.0, 6.0, 0.0, 1.0],
    ]

    # Send waypoints
    waypoint_follower.send_waypoints(waypoints)

    # Wait for completion
    waypoint_follower.wait_for_completion()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
