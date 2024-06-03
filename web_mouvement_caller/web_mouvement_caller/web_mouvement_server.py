# Importar mensajes
from geometry_msgs.msg import Twist
from custom_interface.srv import MyMoveMsg

from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
#importar  biblioteca Python ROS2
import rclpy
from rclpy.node import Node

class Service(Node):
    def __init__(self):
        #constructor con el nombre del nodo
        super().__init__('web_movement_server')
        # declara el objeto servicio pasando como parametros
        # tipo de mensaje
        # nombre del servicio
        # callback del servicio
        self.srv = self.create_service(MyMoveMsg, 'movement', self.my_first_service_callback)

        #inicializar el cliente de la accion ollow_waypoints
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')


        #declara el objeto publisher pasando como parametros
        # tipo de mensaje
        # nombre del topic
        # tamaño de la cola

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)




        #funciones del cliente de la accion 
    def send_goal(self, waypoints):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints
        self.client.wait_for_server()
        self.goal_future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return    
        self.get_logger().info('Goal accepted :)')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()
    def create_pose(self, x, y, w):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = w
        return pose   



    def my_first_service_callback(self, request, response):
        # recibe los parametros de esta clase
        #  recibe el mensaje request
        # devuelve el mensaje response

        # crea un mensaje tipo Twist
        msg = Twist()

        if request.move == "derecha":
            # rellena el mensaje msg con la velocidad angular y lineal
            # necesaria para hacer un giro a la derecha
            waypoints = [
                self.create_pose(1.0, 1.0, 1.0),
                self.create_pose(2.0, 2.0, 1.0),
                self.create_pose(4.0, 4.0, 1.0)
            ]       
            
            # publica el mensaje
            self.publisher.publish(msg)
            # imprime mensaje informando del movimiento
            self.get_logger().info('Girando hacia la derecha')
            self.send_goal(waypoints)
            # devuelve la respuesta
            response.success = True
        elif request.move == "izquierda":
            # rellena el mensaje msg con la velocidad angular y lineal
            # necesaria para hacer un giro a la izquierda
            msg.linear.x = 0.1
            msg.angular.z = 0.5
            # publica el mensaje
            self.publisher.publish(msg)
            # imprime mensaje informando del movimiento
            self.get_logger().info('Girando hacia la izquierda')
            # devuelve la respuesta
            response.success = True
        elif request.move == "delante":
            # rellena el mensaje msg con la velocidad angular y lineal
            # necesaria para moverse hacia delante
            msg.linear.x = 0.1
            msg.angular.z = 0.0
            # publica el mensaje
            self.publisher.publish(msg)
            # imprime mensaje informando del movimiento
            self.get_logger().info('Hacia delante')
            # devuelve la respuesta
            response.success = True
        elif request.move == "atras":
            # rellena el mensaje msg con la velocidad angular y lineal
            # necesaria para moverse hacia atras
            msg.linear.x = -0.1
            msg.angular.z = 0.0
            # publica el mensaje
            self.publisher.publish(msg)
            # imprime mensaje informando del movimiento
            self.get_logger().info('Hacia atras')
            # devuelve la respuesta
            response.success = True
        elif request.move == "parar":
            # rellena el mensaje msg con la velocidad angular y lineal
            # necesaria para parar el robot
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            # publica el mensaje
            self.publisher.publish(msg)
            # imprime mensaje informando del movimiento
            self.get_logger().info('Parando')
            # devuelve la respuesta
            response.success = True
        else:
            # estado de la respuesta
            # si no se ha dado ningun caso anterior
            response.success = False

        # devuelve la respuesta
        return response
    
 
    

def main(args=None):
    # inicializa la comunicacion ROS2
    rclpy.init(args=args)
    # creamos el nodo
    service = Service()
    waypoints = [
        service.create_pose(1.0, 1.0, 1.0),
        service.create_pose(2.0, 2.0, 1.0),
        service.create_pose(4.0, 4.0, 1.0)
    ]
    #waypoints_client.send_goal(waypoints)#self.end_goal(waypoints) dentro de callback
    #service.send_goal(waypoints)
    try:
        #dejamos abierto el servicio
        rclpy.spin(service)
    except KeyboardInterrupt:
        service.get_logger().info('Cerrando el nodo service')
    finally:
        #destruimos el nodo
        service.destroy_node()
        #cerramos la comunicacion
        rclpy.shutdown()
#definimos el ejecutable
if __name__=='__main__':
    main()