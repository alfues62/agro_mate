import sys
import rclpy
sys.path.insert(1, '../agro_mate_nav2_system')
from waypoint_follower import WaypointsClient
from geometry_msgs.msg import PoseStamped

def test_create_pose():
    rclpy.init()  # Inicializar rclpy
    # Crear una instancia de la clase WaypointsClient
    f = WaypointsClient()

    x, y, w = 1.0, 2.0, 3.0
    pose = f.create_pose(x, y, w)

    # Comprobar que el resultado es una instancia de PoseStamped
    assert isinstance(pose, PoseStamped), "El resultado no es una instancia de PoseStamped"

    # Comprobar que los valores son correctos
    assert pose.header.frame_id == 'map', "El frame_id no es correcto"
    assert pose.pose.position.x == x, "El valor de x no es correcto"
    assert pose.pose.position.y == y, "El valor de y no es correcto"
    assert pose.pose.orientation.w == w, "El valor de w no es correcto"

    rclpy.shutdown()  # Asegurarse de que rclpy se cierre correctamente
