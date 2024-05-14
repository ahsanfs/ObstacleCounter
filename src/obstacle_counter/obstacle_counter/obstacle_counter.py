import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray

class ObstacleCounter(Node):
    def __init__(self):
        super().__init__('obstacle_counter')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.path_sub = self.create_subscription(MarkerArray, '/mpc/viz', self.path_callback, 10)
        self.obstacle_coords = []
        self.resolution = None
        self.origin = None
        self.obstacle_hits = 0
        self.previous_path_points = []
        self.path_processed = False

    def map_callback(self, data):
        self.get_logger().info("Map Received")
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
            if marker.color.r == 1.0 and marker.color.g == 0.0 and marker.color.b == 0.0 and marker.color.a == 1.0:
                
                path_points = marker.points
                if not self.is_same_path(path_points):
                    self.get_logger().info("New path (red) is generated")
                    # self.obstacle_hits = 0  # Reset obstacle hit counter for new path
                    self.path_processed = False
                    self.previous_path_points = path_points  # Update the previous path points

                if not self.path_processed:
                    if self.count_obstacle_hits(path_points):
                        self.obstacle_hits += 1
                        self.get_logger().info(f"Obstacle hit detected! Total hits: {self.obstacle_hits}")
                    self.path_processed = True

    def is_same_path(self, path_points):
        if len(path_points) != len(self.previous_path_points):
            return False
        for point1, point2 in zip(path_points, self.previous_path_points):
            if point1.x != point2.x or point1.y != point2.y or point1.z != point2.z:
                return False
        return True

    def count_obstacle_hits(self, path_points):
        for point in path_points:
            if self.check_collision(point):
                return True
        return False

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
