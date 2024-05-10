import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints

class WaypointFollowerClient(Node):
    def __init__(self):
        super().__init__('waypoint_follower_client')
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

    def send_goal(self):
        self.get_logger().info('Sending waypoints...')
        goal_msg = FollowWaypoints.Goal()

        
        # Create a list of poses as PoseStamped objects
        waypoint1 = PoseStamped()
        waypoint1.pose.position.x = 1.0
        waypoint1.pose.position.y = 1.0
        waypoint1.pose.orientation.w = 1.0
        waypoint1.header.frame_id = "map"

        waypoint2 = PoseStamped()
        waypoint2.pose.position.x = 2.0
        waypoint2.pose.position.y = 2.0
        waypoint2.pose.orientation.w = 1.0

        waypoint2.header.frame_id = "map"

        waypoint3 = PoseStamped()
        waypoint3.pose.position.x = 3.0
        waypoint3.pose.position.y = 3.0
        waypoint3.pose.orientation.w = 1.0

        waypoint2.header.frame_id = "map"

        # Add waypoints to the goal message
        goal_msg.poses = [waypoint1, waypoint2, waypoint3]

        # Send the goal to the action server
        self.action_client.wait_for_server()
        send_goal_future = self.action_client.send_goal_async(goal_msg)

        # Register callbacks for feedback and results
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected')
            return

        self.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Waypoints completed successfully')
        else:
            self.get_logger().info('Failed to complete waypoints')
            for missed in result.missed_waypoints:
                self.get_logger().info(f"Missed waypoint at index: {missed.index}")

    def feedback_callback(self, feedback):
        current_waypoint = feedback.feedback.current_waypoint
        self.get_logger().info(f'Current waypoint: {current_waypoint}')


def main(args=None):
    rclpy.init(args=args)
    client = WaypointFollowerClient()
    client.send_goal()
    rclpy.spin(client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
