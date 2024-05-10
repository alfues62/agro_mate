import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from agro_mate_interface.srv import MyPointMsg

class NavigationService(Node):
    def __init__(self):
        super().__init__('point_server')
        
        # Crear el servicio que recibe coordenadas y las publica como PoseStamped
        self.srv = self.create_service(MyPointMsg, 'goal_pose', self.navigation_callback)
        #self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Crear el subscriber para escuchar datos enviados desde la web u otros nodos
        self.subscriber = self.create_subscription(
            String, 
            'goal_pose',  # Tópico en el que se espera recibir mensajes
            self.web_command_callback, 
            10
        )

    def navigation_callback(self, request, response):
        x = request.x
        y = request.y
        theta = request.theta  # Orientación opcional
        
        # Crear el mensaje de destino
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()  # Añadir la marca de tiempo
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = theta  # Opcional
        pose.pose.orientation.w = 1.0  # Orientación predeterminada

        # Publicar la posición
        self.publisher.publish(pose)

        response.success = True
        self.get_logger().info(f'Enviando coordenadas: x={x}, y={y}, theta={theta}')

        return response

    def web_command_callback(self, msg):
        # Procesar el mensaje recibido desde la web u otros nodos
        self.get_logger().info(f'Recibido mensaje: {msg.data}')
        
        # Haz algo con el mensaje, como cambiar el comportamiento del nodo o actuar en consecuencia
        if msg.data == 'cancel_navigation':
            # Aquí podrías detener la navegación, por ejemplo
            self.get_logger().info("Cancelando la navegación")
            # Publicar un mensaje para detener el robot o cancelar la acción
            # (Esto depende de tu arquitectura de control)
        else:
            # Maneja otros casos según tus necesidades
            self.get_logger().info("Comando no reconocido.")


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