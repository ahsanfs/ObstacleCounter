import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray

class ObstacleCounter(Node):
    def __init__(self):
        super().__init__('obstacle_counter')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.path_sub = self.create_subscription(MarkerArray, '/mpc_path', self.path_callback, 10)
        self.obstacle_coords = []
        self.resolution = None
        self.origin = None
        self.obstacle_hits = 0

    def map_callback(self, data):
        self.resolution = data.info.resolution
        self.origin = data.info.origin.position
        width = data.info.width
        height = data.info.height

        self.obstacle_coords.clear()

        for y in range(height):
            for x in range(width):
                index = x + y * width
                if data.data[index] > 50:  # Occupied cell
                    ox = self.origin.x + x * self.resolution
                    oy = self.origin.y + y * self.resolution
                    self.obstacle_coords.append((ox, oy))

    def path_callback(self, data):
        for marker in data.markers:
            path_points = marker.points
            for point in path_points:
                if self.check_collision(point):
                    self.obstacle_hits += 1
                    self.get_logger().info(f"Obstacle hit detected! Total hits: {self.obstacle_hits}")

    def check_collision(self, point):
        for (ox, oy) in self.obstacle_coords:
            if self.is_point_near_obstacle(point, ox, oy):
                return True
        return False

    def is_point_near_obstacle(self, point, ox, oy, threshold=0.1):
        distance = ((point.x - ox) ** 2 + (point.y - oy) ** 2) ** 0.5
        return distance < threshold

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleCounter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

