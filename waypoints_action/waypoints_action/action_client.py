import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints  # Importar el tipo de acción para la navegación
from geometry_msgs.msg import PoseStamped  # Importar el tipo de mensaje para el goal

class MyActionClient(Node):
    def __init__(self):
        super().__init__('my_action_client')
        self._action_client = ActionClient(self, FollowWaypoints, 'waypoints')  # Configura el cliente de acción

    def send_goal(self, waypoints):
        self._action_client.wait_for_server()  # Espera a que el servidor esté listo

        goal_msg = FollowWaypoints.Goal()  # Crear el mensaje para el objetivo
        goal_msg.FollowWaypoints = [self.create_pose_stamped(x, y) for x, y in waypoints]

        # Enviar el objetivo y establecer callbacks para feedback y resultado
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def create_pose_stamped(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'  # Configura el frame ID
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0  # Orientación por defecto
        return pose

    def feedback_callback(self, feedback_msg):
        current_waypoint = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f"Current waypoint: {current_waypoint}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal was rejected.")
            return

        self.get_logger().info("Goal accepted.")

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()

        if result is None:
            self.get_logger().warn("No result received.")
        elif result.success:
            self.get_logger().info(f"Success! {result.message}")
        else:
            self.get_logger().warn(f"Failed! {result.message}")

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    action_client = MyActionClient()
    action_client.send_goal([[2.0, 2.5], [1.1, 0.5], [1.0, 1.5]])  # Waypoints para enviar

    rclpy.spin(action_client)  # Mantener el cliente corriendo para recibir feedback y resultado

if __name__ == '__main__':
    main()