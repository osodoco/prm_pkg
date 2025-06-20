import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from visualization_msgs.msg import Marker
from nav2_msgs.action import FollowWaypoints

import numpy as np
import networkx as nx

from rclpy.action import ActionClient


class PRMNavigator(Node):
    def __init__(self):
        super().__init__('prm_navigation')

        # Subscripciones para recibir PRM y goal_pose
        self.create_subscription(Marker, 'prm_nodes', self.prm_nodes_callback, 10)
        self.create_subscription(Marker, 'prm_edges', self.prm_edges_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        # Acción para enviar a Nav2
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        # Publicador para inicializar la pose inicial en Nav2
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # 👉 Publicador de Marker para visualizar el path A*
        self.path_marker_pub = self.create_publisher(Marker, 'prm_astar_path', 10)

        # Estado interno
        self.nodes = []
        self.edges = []
        self.graph = nx.Graph()

        self.current_pose = None  # pose actual en 'map'
        self.goal_pose = None     # meta recibida por topic

        # Publicar la pose inicial una vez
        self.publish_initial_pose()

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0  # sin rotación
        self.current_pose = (0.0, 0.0)  # actualizar estado interno también

        self.initial_pose_pub.publish(msg)
        self.get_logger().info("Pose inicial (0,0,0) publicada en /initialpose.")

    def prm_nodes_callback(self, msg):
        self.nodes = [(p.x, p.y) for p in msg.points]
        self.get_logger().info(f"Recibidos {len(self.nodes)} nodos PRM")
        self.rebuild_graph()

    def prm_edges_callback(self, msg):
        pts = msg.points
        self.edges = []
        for i in range(0, len(pts), 2):
            p1 = (pts[i].x, pts[i].y)
            p2 = (pts[i+1].x, pts[i+1].y)
            i1 = self.closest_node_idx(p1)
            i2 = self.closest_node_idx(p2)
            if i1 != i2:
                self.edges.append((i1, i2))
        self.get_logger().info(f"Recibidas {len(self.edges)} aristas PRM")
        self.rebuild_graph()

    def goal_pose_callback(self, msg):
        self.goal_pose = (
            msg.pose.position.x,
            msg.pose.position.y
        )
        self.get_logger().info(f"Meta recibida: {self.goal_pose}")
        self.plan_and_send()

    def rebuild_graph(self):
        if self.nodes and self.edges:
            self.graph.clear()
            for idx, p in enumerate(self.nodes):
                self.graph.add_node(idx, pos=p)
            self.graph.add_edges_from(self.edges)
            self.get_logger().info(f"Grafo PRM reconstruido con {len(self.graph.nodes)} nodos y {len(self.graph.edges)} aristas")

    def closest_node_idx(self, pos):
        if not self.nodes:
            return None
        nodes = np.array(self.nodes)
        dists = np.linalg.norm(nodes - np.array(pos), axis=1)
        return np.argmin(dists)

    def plan_and_send(self):
        if not self.graph or self.current_pose is None or self.goal_pose is None:
            self.get_logger().warn("Esperando grafo, pose inicial o goal...")
            return

        start_idx = self.closest_node_idx(self.current_pose)
        goal_idx = self.closest_node_idx(self.goal_pose)

        self.get_logger().info(f"Nodo inicio: {start_idx}, Nodo goal: {goal_idx}")

        try:
            idx_path = nx.astar_path(
                self.graph,
                source=start_idx,
                target=goal_idx,
                heuristic=lambda a, b: np.linalg.norm(
                    np.array(self.graph.nodes[a]['pos']) - np.array(self.graph.nodes[b]['pos'])
                )
            )
        except nx.NetworkXNoPath:
            self.get_logger().error("No hay camino posible en el PRM")
            return

        self.get_logger().info(f"Camino A* con {len(idx_path)} nodos")

        # 📌 Publicar el camino como Marker en RViz
        self.publish_path_marker(idx_path)

        # Crear poses para FollowWaypoints
        poses = []
        for i in idx_path:
            x, y = self.nodes[i]
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            poses.append(pose)

        self.action_client.wait_for_server()
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.get_logger().info("Enviando camino como acción FollowWaypoints...")
        self.action_client.send_goal_async(goal_msg)
        self.get_logger().info("Acción enviada.")

    def publish_path_marker(self, idx_path):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'prm_astar_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # grosor de la línea
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0  # verde

        marker.points = []
        for i in idx_path:
            x, y = self.nodes[i]
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        self.path_marker_pub.publish(marker)
        self.get_logger().info("Marker del camino A* publicado en topic 'prm_astar_path'.")

def main(args=None):
    rclpy.init(args=args)
    node = PRMNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

