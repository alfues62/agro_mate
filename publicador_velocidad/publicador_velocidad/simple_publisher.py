import rclpy
# importamos las librerias ROS2 de python 
from rclpy.node import Node
# importamos los mensajes tipo PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovarianceStamped


# creamos una clase pasándole como parámetro el Nodo
class SimplePublisher(Node):

    def __init__(self):
        # Constructor de la clase
        # ejecutamos super() para inicializar el Nodo
        # introducimos le nombre del nodo como parámetro
        super().__init__('simple_publisher')
        # creamos el objeto publisher
        # que publicara en el topic /cmd_vel 
        # la cola del topic es de 10 mensajes
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        # definimos un periodo para publicar periodicamente
        timer_period = 1.5
        # creamos un timer con dos parametros:
        # - el periodo (0.5 seconds)
        # - la funcion a realizar  (timer_callback)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # creamos el mensaje tipo Twist
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp.sec = 123456  # Example seconds value
        msg.header.stamp.nanosec = 789000000  # Example nanoseconds value
        msg.pose.pose.position.x = 1.2
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance = [1.0] * 36  # Example covariance data
        self.publisher_.publish(msg)
        # Mostramos el mensaje por el terminal
        self.get_logger().info('Publishing: "%s"' % msg)
            
def main(args=None):
    # inicializa la comunicación
    rclpy.init(args=args)
    # declara el constructor del nodo 
    simple_publisher = SimplePublisher()
    # dejamos vivo el nodo
    # para parar el programa habrá que matar el node (ctrl+c)
    rclpy.spin(simple_publisher)
    # destruye en nodo
    simple_publisher.destroy_node()
    # se cierra la comunicacion ROS
    rclpy.shutdown()

if __name__ == '__main__':
    main()