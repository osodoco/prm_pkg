import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import numpy as np
import yaml
import cv2
import os
import math


class PRMGenerator(Node):
    def __init__(self):
        super().__init__('generate_prm')

        self.declare_parameter('num_nodes', 500)
        self.declare_parameter('connection_radius', 0.55)
        self.num_nodes = self.get_parameter('num_nodes').get_parameter_value().integer_value
        self.connection_radius = self.get_parameter('connection_radius').get_parameter_value().double_value

        # Ruta absoluta o usa get_package_share_directory si prefieres
        map_yaml = '/home/osdoco/prm_ws/src/prm_pkg/maps/coppeliasim_map.yaml'

        with open(map_yaml, 'r') as f:
            map_data = yaml.safe_load(f)

        self.map_resolution = map_data['resolution']
        self.map_origin = map_data['origin'][:2]
        map_image_path = os.path.join(os.path.dirname(map_yaml), map_data['image'])

        #Leer como grayscale + binarizar fuerte
        raw_img = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
        _, self.map_img = cv2.threshold(raw_img, 250, 255, cv2.THRESH_BINARY)

        # Guardar imagen umbralizada para diagnóstico
        cv2.imwrite("thresholded_map.png", self.map_img)

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

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.publisher_nodes = self.create_publisher(Marker, 'prm_nodes', qos_profile)
        self.publisher_edges = self.create_publisher(Marker, 'prm_edges', qos_profile)

        self.generate_prm()

    def is_in_obstacle(self, point):
        x, y = point
        px = int((x - self.map_origin[0]) / self.map_resolution)
        py = int((y - self.map_origin[1]) / self.map_resolution)
        
        py = self.map_img.shape[0] - py - 1

        # Verificar límites de la imagen
        if px < 0 or py < 0 or px >= self.map_img.shape[1] or py >= self.map_img.shape[0]:
            return True

        # Verificar el píxel y un área alrededor (para mayor robustez)
        size = 1  # Radio del área a verificar
        for i in range(-size, size+1):
            for j in range(-size, size+1):
                if 0 <= px+i < self.map_img.shape[1] and 0 <= py+j < self.map_img.shape[0]:
                    if self.map_img[py+j, px+i] < 250:  # Más estricto que == 0
                        return True
        return False

    def is_path_blocked(self, p1, p2, step=None):
        if step is None:
            # Paso adaptativo basado en la resolución del mapa
            step = self.map_resolution / 2.0
        
        vec = np.array(p2) - np.array(p1)
        dist = np.linalg.norm(vec)
        if dist == 0:
            return False
        
        direction = vec / dist
        num_steps = math.ceil(dist / step)
        
        # Verificar puntos intermedios
        for i in np.linspace(0, dist, num_steps + 1):
            pt = np.array(p1) + i * direction
            if self.is_in_obstacle(pt):
                self.get_logger().debug(f"Camino bloqueado en punto {pt}")
                return True
        
        # Verificación adicional en los extremos
        if self.is_in_obstacle(p1) or self.is_in_obstacle(p2):
            self.get_logger().debug(f"Extremo del camino está en obstáculo: p1={p1}, p2={p2}")
            return True
            
        return False

    def visualize_problematic_connections(self, nodes, edges):
        # Crear una imagen de diagnóstico
        diag_img = cv2.cvtColor(self.map_img, cv2.COLOR_GRAY2BGR)
        
        # Escalar la imagen para mejor visualización (3x más grande)
        scale_factor = 3
        diag_img = cv2.resize(diag_img, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_NEAREST)
        
        # Dibujar todos los nodos (verdes)
        node_radius = int(4 * scale_factor)
        for x, y in nodes:
            px = int(((x - self.map_origin[0]) / self.map_resolution) * scale_factor)
            py = int(((y - self.map_origin[1]) / self.map_resolution) * scale_factor)
            cv2.circle(diag_img, (px, py), node_radius, (0, 255, 0), -1)
        
        # Dibujar conexiones (rojas)
        line_thickness = int(2 * scale_factor)
        for i, j in edges:
            p1 = nodes[i]
            p2 = nodes[j]
            px1 = int(((p1[0] - self.map_origin[0]) / self.map_resolution) * scale_factor)
            py1 = int(((p1[1] - self.map_origin[1]) / self.map_resolution) * scale_factor)
            px2 = int(((p2[0] - self.map_origin[0]) / self.map_resolution) * scale_factor)
            py2 = int(((p2[1] - self.map_origin[1]) / self.map_resolution) * scale_factor)
            cv2.line(diag_img, (px1, py1), (px2, py2), (0, 0, 255), line_thickness)
        
        # Mostrar imagen en ventana grande
        window_name = "PRM Diagnosis - Cierra esta ventana manualmente para continuar"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1200, 900)  # Tamaño fijo grande
        cv2.imshow(window_name, diag_img)
        
        # Guardar imagen de diagnóstico
        cv2.imwrite("prm_diagnosis.png", diag_img)
        
        # Esperar indefinidamente hasta que el usuario cierre la ventana manualmente
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def generate_prm(self):
        xmin, ymin, xmax, ymax = self.bounds
        nodes = []
        attempt = 0
        max_attempts = self.num_nodes * 10

        while len(nodes) < self.num_nodes and attempt < max_attempts:
            x = np.random.uniform(xmin, xmax)
            y = np.random.uniform(ymin, ymax)
            if not self.is_in_obstacle((x, y)):
                nodes.append((x, y))
            attempt += 1

        edges = []
        for i, p1 in enumerate(nodes):
            for j, p2 in enumerate(nodes):
                if i >= j:
                    continue
                if np.linalg.norm(np.array(p1) - np.array(p2)) < self.connection_radius:
                    if not self.is_path_blocked(p1, p2):
                        self.get_logger().debug(f"Conexión válida entre {p1} y {p2}")
                        edges.append((i, j))
                    else:
                        self.get_logger().debug(f"Conexión bloqueada entre {p1} y {p2}")

        self.get_logger().info(f"Generated {len(nodes)} nodes and {len(edges)} edges.")
        
        # Visualización de diagnóstico
        self.visualize_problematic_connections(nodes, edges)
        
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
