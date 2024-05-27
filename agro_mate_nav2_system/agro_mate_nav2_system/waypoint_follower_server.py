import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped

class FollowWaypointsServer(Node):

    def __init__(self):
        super().__init__('follow_waypoints_server')
        self._action_server = ActionServer(
            self,
            FollowWaypoints,
            'follow_waypoints',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Following waypoints...')
        waypoints = goal_handle.request.poses

        # Simulate following each waypoint
        for waypoint in waypoints:
            self.get_logger().info(f'Navigating to waypoint at position: {waypoint.pose.position}')
            # Here you would add your logic to handle navigation to the waypoint

        goal_handle.succeed()
        return FollowWaypoints.Result()

def main(args=None):
    rclpy.init(args=args)
    server = FollowWaypointsServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
