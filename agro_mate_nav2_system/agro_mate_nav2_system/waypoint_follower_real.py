import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints

class WaypointsClient(Node):

    def __init__(self):
        super().__init__('waypoint_follower')
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')


    def send_goal(self, waypoints):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints
        self.client.wait_for_server()
        self.goal_future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()

    def create_pose(self, x, y, w):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = w
        return pose

def main(args=None):
    rclpy.init(args=args)
    waypoints_client = WaypointsClient()
    waypoints = [
        waypoints_client.create_pose(0.6, 0.1, 1.0),
        waypoints_client.create_pose(0.78, -1.02, 1.0),
        waypoints_client.create_pose(0.78, -1.14, 1.0)
    ]
    waypoints_client.send_goal(waypoints)#self.end_goal(waypoints) dentro de callback 

    rclpy.spin(waypoints_client)

if __name__ == '__main__':
    main()
