import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose  # Importar el tipo de acción para la navegación
from geometry_msgs.msg import PoseStamped  # Para trabajar con el goal pose

class MyActionServer(Node):
    def __init__(self):
        super().__init__('my_action_server')
        # Crear el servidor de acción para 'NavigateToPose'
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback,  # Función que maneja el goal
            cancel_callback=self.cancel_callback  # Función para manejar cancelaciones
        )

    def execute_callback(self, goal_handle):
        # Obtenemos el goal del cliente
        goal = goal_handle.request
        pose = goal.pose  # La posición de destino

        # Proporcionamos feedback periódico mientras el goal está en progreso
        self.get_logger().info(f'Goal received: {pose.pose.position}')

        # Aquí puedes implementar la lógica para llevar al robot a la posición indicada.
        # Por simplicidad, usaremos un bucle simulado para emular el progreso.
        import time
        for i in range(10):
            time.sleep(0.5)  # Simulamos un retraso
            # Proporcionar feedback
            feedback_msg = NavigateToPose.Feedback()
            feedback_msg.feedback = f'Progress: {i * 10}%'
            goal_handle.publish_feedback(feedback_msg)

        # Indicamos que el goal se completó
        goal_handle.succeed()

        # Devolver el resultado
        result = NavigateToPose.Result()
        result.result = 'Navigation Complete'
        return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Goal canceled')
        return rclpy.action.GoalResponse.CANCELLED


def main(args=None):
    rclpy.init(args=args)

    action_server = MyActionServer()

    rclpy.spin(action_server)  # Mantiene el servidor ejecutándose hasta que se detenga


if __name__ == '__main__':
    main()