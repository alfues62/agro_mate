import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from agro_mate_interface.srv import MyPointMsg
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class NavigationService(Node):
    def __init__(self):
        super().__init__('point_server')
        # Crear el servicio que recibe las coordenadas x y y
        self.srv = self.create_service(MyPointMsg, 'point_server', self.navigation_callback)
        # Cliente de acción para enviar objetivos de navegación
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def navigation_callback(self, request, response):
        x = request.x
        y = request.y
        
        # Crear un objetivo para NavigateToPose
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        # Mantener la orientación por defecto
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        # Esperar al servidor de acción
        if not self.action_client.wait_for_server(10):  # 10 segundos de espera
            self.get_logger().error("Servidor de acción no disponible")
            response.success = False
            return response

        # Enviar el objetivo al cliente de acción
        self.action_client.send_goal_async(goal)
        self.get_logger().info(f'Objetivo enviado: x={x}, y={y}')

        # Establecer la respuesta del servicio
        response.success = True
        return response

def main():
    rclpy.init()
    navigation_service = NavigationService()

    try:
        rclpy.spin(navigation_service)
    except KeyboardInterrupt:
        navigation_service.get_logger().info("Cerrando el nodo de servicio")
    finally:
        navigation_service.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
