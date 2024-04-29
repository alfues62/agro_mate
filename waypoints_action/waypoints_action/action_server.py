import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints  # Importar el tipo de acción para la navegación
from geometry_msgs.msg import PoseStamped  # Para trabajar con el goal pose
from time import sleep


class MyActionServer(Node):
    def __init__(self):
        super().__init__('my_action_server')
        # Crear el servidor de acción para 'FollowWaypoints'
        self._action_server = ActionServer(
            self,
            FollowWaypoints,
            'follow_waypoints',
            execute_callback=self.execute_callback,  # Función que maneja el goal
            goal_callback=self.goal_callback,  # Callback para aceptar el objetivo
        )

    def goal_callback(self, goal_request):
        # Puedes agregar validación para el objetivo si es necesario
        return rclpy.action.GoalResponse.ACCEPT  # Aceptar el objetivo

    def execute_callback(self, goal_handle):
        # Obtener los waypoints del objetivo
        waypoints = goal_handle.request.waypoints
        number_of_waypoints = len(waypoints)

        # Procesar cada waypoint y proporcionar feedback
        for index, waypoint in enumerate(waypoints):
            # Simular la ejecución del waypoint (aquí puedes agregar lógica para mover el robot)
            self.get_logger().info(f"Procesando waypoint {index + 1}/{number_of_waypoints}")

            # Proporcionar feedback al cliente
            feedback_msg = FollowWaypoints.Feedback()
            feedback_msg.current_waypoint = index  # Índice del waypoint actual
            goal_handle.publish_feedback(feedback_msg)

            # Simular tiempo para alcanzar el waypoint
            sleep(1)  # Pausa para simular la ejecución de cada waypoint

        # Al finalizar, proporcionar el resultado al cliente
        result = FollowWaypoints.Result()
        result.success = True  # Indicar que fue exitoso
        result.message = "Todos los waypoints fueron alcanzados exitosamente."
        goal_handle.succeed(result)

        return result  # Devolver el resultado

def main(args=None):
    rclpy.init(args=args)

    action_server = MyActionServer()  # Crear el servidor de acción
    rclpy.spin(action_server)  # Mantener el servidor corriendo para procesar solicitudes

if __name__ == '__main__':
    main()