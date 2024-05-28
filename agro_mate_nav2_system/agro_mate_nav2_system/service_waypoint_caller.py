import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from example_interfaces.action import Fibonacci
from example_interfaces.srv import Trigger

class Service_waypoint_caller(Node):
    def __init__(self):
        super().__init__('service_waypoint_caller')
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.srv = self.create_service(Trigger, 'service_waypoint_caller', self.trigger_fibonacci_callback)

    def trigger_fibonacci_callback(self, request, response):
        self.get_logger().info('Received service call, sending action request...')
        
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10  # Example order, adjust as needed
        
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
        response.success = True
        return response

    

    #action client operations
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()

    def send_goal(self, waypoints):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints
        self.action_client.wait_for_server()
        self.goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.goal_future.add_done_callback(self.goal_response_callback)

    def create_pose(self, x, y, w):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = w
        return pose    

def main(args=None):
    rclpy.init(args=args)
    service_waypoint_caller = Service_waypoint_caller()
    waypoints = [
        service_waypoint_caller.create_pose(1.0, 1.0, 1.0),
        service_waypoint_caller.create_pose(2.0, 2.0, 1.0),
        service_waypoint_caller.create_pose(4.0, 4.0, 1.0)
    ]
    service_waypoint_caller.send_goal(waypoints)
    executor = MultiThreadedExecutor()
    rclpy.spin(service_waypoint_caller, executor=executor)

    service_waypoint_caller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
