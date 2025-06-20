import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import numpy as np
import yaml
import cv2
import os
from ament_index_python.packages import get_package_share_directory


class PRMGenerator(Node):
    def __init__(self):
        super().__init__('generate_prm')

        # Leer parámetros PRM
        self.declare_parameter('num_nodes', 200)
        self.declare_parameter('connection_radius', 1.0)
        self.num_nodes = self.get_parameter('num_nodes').get_parameter_value().integer_value
        self.connection_radius = self.get_parameter('connection_radius').get_parameter_value().double_value

        # Leer mapa YAML
        map_yaml = '/home/osdoco/prm_ws/src/prm_pkg/maps/coppeliasim_map.yaml'

        with open(map_yaml, 'r') as f:
            map_data = yaml.safe_load(f)

        self.map_resolution = map_data['resolution']
        self.map_origin = map_data['origin'][:2]
        map_image_path = os.path.join(os.path.dirname(map_yaml), map_data['image'])

        self.map_img = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
        if self.map_img is None:
            self.get_logger().error(f"Could not load map image at {map_image_path}")
            exit(1)

        height, width = self.map_img.shape
        self.bounds = [
            self.map_origin[0],
            self.map_origin[1],
            self.map_origin[0] + width * self.map_resolution,
            self.map_origin[1] + height * self.map_resolution
        ]

        self.get_logger().info(f"Map loaded: size={width}x{height}, resolution={self.map_resolution}")

        # QoS Transient Local: asegura que los suscriptores tardíos reciben el último mensaje
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.publisher_nodes = self.create_publisher(Marker, 'prm_nodes', qos_profile)
        self.publisher_edges = self.create_publisher(Marker, 'prm_edges', qos_profile)

        # Generar PRM
        self.generate_prm()

    def is_in_obstacle(self, point):
        x, y = point
        px = int((x - self.map_origin[0]) / self.map_resolution)
        py = int((y - self.map_origin[1]) / self.map_resolution)

        if px < 0 or py < 0 or px >= self.map_img.shape[1] or py >= self.map_img.shape[0]:
            return True  # fuera de mapa

        return self.map_img[py, px] < 250  # umbral: blanco = libre

    def is_path_blocked(self, p1, p2, step=0.005):
        vec = np.array(p2) - np.array(p1)
        dist = np.linalg.norm(vec)
        if dist == 0:
            return False
        direction = vec / dist
        for i in np.arange(0, dist, step):
            pt = np.array(p1) + i * direction
            if self.is_in_obstacle(pt):
                return True
        return False

    def generate_prm(self):
        xmin, ymin, xmax, ymax = self.bounds
        nodes = []
        while len(nodes) < self.num_nodes:
            x = np.random.uniform(xmin, xmax)
            y = np.random.uniform(ymin, ymax)
            if not self.is_in_obstacle((x, y)):
                nodes.append((x, y))

        edges = []
        for i, p1 in enumerate(nodes):
            for j, p2 in enumerate(nodes):
                if i >= j:
                    continue
                if np.linalg.norm(np.array(p1) - np.array(p2)) < self.connection_radius:
                    if not self.is_path_blocked(p1, p2):
                        edges.append((i, j))

        self.get_logger().info(f"Generated {len(nodes)} nodes and {len(edges)} edges.")

        self.publish_nodes(nodes)
        self.publish_edges(nodes, edges)

    def publish_nodes(self, nodes):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = 'prm_nodes'
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.points = [Point(x=x, y=y, z=0.0) for x, y in nodes]
        self.publisher_nodes.publish(marker)

    def publish_edges(self, nodes, edges):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = 'prm_edges'
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        for i, j in edges:
            marker.points.append(Point(x=nodes[i][0], y=nodes[i][1], z=0.0))
            marker.points.append(Point(x=nodes[j][0], y=nodes[j][1], z=0.0))
        self.publisher_edges.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PRMGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

