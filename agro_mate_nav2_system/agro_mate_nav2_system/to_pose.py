import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose  # Importación de la acción de navegación
from geometry_msgs.msg import PoseStamped  # Importación del tipo de mensaje para el goal


class MyActionClient(Node):
    def __init__(self):
        super().__init__('my_action_client')
        # Crear el cliente de acción para 'NavigateToPose'
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y):
        # Crear el mensaje de goal y establecer la posición
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # Orientación hacia adelante

        # Esperar a que el servidor de acción esté listo
        self._action_client.wait_for_server()

        # Enviar el goal y configurar los callbacks
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')


def main(args=None):
    rclpy.init(args=args)

    # Crear el cliente de acción y enviar el goal para la posición (x, y)
    action_client = MyActionClient()
    action_client.send_goal(2.5, -0.5)  # Posición a la que queremos que el robot navegue

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()