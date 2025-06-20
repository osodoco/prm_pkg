import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped

class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')

        # Crea cliente de acción para FollowWaypoints
        self._client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        # Espera servidor de acción
        self._client.wait_for_server()

        # Define los dos waypoints exactamente como en tu ejemplo
        poses = []

        # Waypoint 1
        pose1 = PoseStamped()
        pose1.header.frame_id = 'map'
        pose1.pose.position.x = 0.5
        pose1.pose.position.y = 0.5
        pose1.pose.position.z = 0.0
        pose1.pose.orientation.w = 1.0
        poses.append(pose1)

        # Waypoint 2
        pose2 = PoseStamped()
        pose2.header.frame_id = 'map'
        pose2.pose.position.x = -0.5
        pose2.pose.position.y = -0.5
        pose2.pose.position.z = 0.0
        pose2.pose.orientation.z = 0.7071
        pose2.pose.orientation.w = 0.7071
        poses.append(pose2)

        # Crea mensaje Goal
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        # Enviar goal
        self.get_logger().info("Enviando FollowWaypoints goal...")
        self._send_goal_future = self._client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rechazado.')
            return

        self.get_logger().info('Goal aceptado.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Resultado recibido: {result}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSender()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

