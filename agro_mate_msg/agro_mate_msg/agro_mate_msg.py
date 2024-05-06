import rclpy  # importar ROS2 python libraries
from rclpy.node import Node  # importar la clase Node
from custom_interface.msg import Distvel  # importar Distvel del módulo custom_interface

def main(args=None):
    # Inicializa la comunicación con ROS2
    rclpy.init(args=args)

    # Declara el nodo
    node = Node('my_showmsg_node')

    # Declara el mensaje
    my_distvel = Distvel()
    my_distvel.distancia = 1
    my_distvel.velocidad = 0.1

    try:
        # Imprime el mensaje por pantalla
        node.get_logger().info('El mensaje recogido es:')
        node.get_logger().info('distancia = %d' % my_distvel.distancia)
        node.get_logger().info('velocidad = %f' % my_distvel.velocidad)

        # Deja el nodo abierto hasta ctrl+c
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        # Mensaje al cerrar el nodo
        node.get_logger().info('Cerrando el nodo...')
    
    finally:
        # Destruye el nodo
        node.destroy_node()

        # Cierra la comunicación ROS2
        rclpy.shutdown()

if __name__ == '__main__':
    main()
