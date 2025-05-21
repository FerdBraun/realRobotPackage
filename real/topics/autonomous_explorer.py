#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import math
import numpy as np

class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_node')

        # Подписки
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10
        )

        # Публикатор целевых точек
        self.goal_pub = self.create_publisher(
            PointStamped,
            '/clicked_point',  # Ваш планировщик ожидает цели здесь
            10
        )

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Переменные
        self.costmap = None
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.map_width = 0
        self.map_height = 0

        # Таймер для выбора целей
        self.timer = self.create_timer(3.0, self.select_goal)  # Каждые 5 секунд выбираем новую цель

    def costmap_callback(self, msg):
        """Обработчик карты стоимости"""
        self.costmap = msg
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.get_logger().info('Costmap updated')

    def select_goal(self):
        """Выбор новой цели для исследования"""
        if self.costmap is None:
            self.get_logger().warn('Costmap not received yet')
            return

        # Преобразуем карту в массив NumPy
        data = np.array(self.costmap.data, dtype=np.int8).reshape((self.map_height, self.map_width))

        # Находим границы между известной и неизвестной территорией
        frontier_points = self.find_frontiers(data)

        if len(frontier_points) == 0:
            self.get_logger().info('No more frontiers to explore. Exploration complete.')
            self.timer.cancel()  # Останавливаем таймер
            return

        # Выбираем ближайшую к роботу точку фронтира
        robot_position = self.get_robot_position()
        closest_point = self.find_closest_frontier(robot_position, frontier_points)

        # Конвертируем точку в мировые координаты
        goal_x, goal_y = self.grid_to_world(*closest_point)

        # Проверяем достижимость и безопасность
        if not self.is_reachable(goal_x, goal_y) or not self.is_safe(goal_x, goal_y):
            self.get_logger().warn(f'Goal ({goal_x}, {goal_y}) is not reachable or safe. Skipping.')
            return

        # Публикуем выбранную цель
        goal = PointStamped()
        goal.header.frame_id = 'map'
        goal.point.x, goal.point.y = goal_x, goal_y
        goal.point.z = 0.0

        self.goal_pub.publish(goal)
        self.get_logger().info(f'New exploration goal: ({goal_x}, {goal_y})')

    def find_frontiers(self, data):
        """Находит точки фронтира (границы между известной и неизвестной территорией)."""
        frontiers = []
        for y in range(1, self.map_height - 1):
            for x in range(1, self.map_width - 1):
                if data[y, x] == 0:  # Свободная клетка
                    neighbors = [
                        data[y + dy, x + dx]
                        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]
                    ]
                    if any(n == -1 for n in neighbors):  # Есть соседняя неизвестная клетка
                        wx, wy = self.grid_to_world(x, y)
                        if self.is_reachable(wx, wy) and self.is_safe(wx, wy):  # Проверяем достижимость и безопасность
                            frontiers.append((x, y))
        return frontiers

    def find_closest_frontier(self, robot_position, frontiers):
        """Находит ближайшую к роботу точку фронтира."""
        robot_x, robot_y = self.world_to_grid(robot_position.point.x, robot_position.point.y)
        closest_point = None
        min_distance = float('inf')

        for fx, fy in frontiers:
            distance = math.hypot(fx - robot_x, fy - robot_y)
            if distance < min_distance:
                min_distance = distance
                closest_point = (fx, fy)

        return closest_point

    def get_robot_position(self):
        """Получение текущей позиции робота в системе координат map."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            robot_position = PointStamped()
            robot_position.header.frame_id = 'base_link'
            robot_position.point.x = 0.0
            robot_position.point.y = 0.0
            robot_position.point.z = 0.0

            transformed_position = do_transform_point(robot_position, transform)
            return transformed_position

        except Exception as e:
            self.get_logger().error(f'Error in getting robot position: {str(e)}')
            raise

    def world_to_grid(self, wx, wy):
        """Конвертация мировых координат в координаты сетки."""
        mx = int((wx - self.origin_x) / self.resolution)
        my = int((wy - self.origin_y) / self.resolution)
        return (mx, my)

    def grid_to_world(self, mx, my):
        """Конвертация координат сетки в мировые координаты."""
        wx = self.origin_x + mx * self.resolution
        wy = self.origin_y + my * self.resolution
        return (wx, wy)

    def is_reachable(self, x, y):
        """Проверяет, является ли точка (x, y) достижимой."""
        grid_x, grid_y = self.world_to_grid(x, y)
        if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
            cost = self.costmap.data[grid_y * self.map_width + grid_x]
            return cost == 0  # Свободная клетка
        return False

    def is_safe(self, x, y, buffer=0.3):
        """Проверяет, находится ли точка (x, y) на безопасном расстоянии от препятствий."""
        grid_x, grid_y = self.world_to_grid(x, y)
        radius = int(buffer / self.resolution)  # Буфер в клетках
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                nx, ny = grid_x + dx, grid_y + dy
                if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                    cost = self.costmap.data[ny * self.map_width + nx]
                    if cost > 0:  # Занятая или опасная клетка
                        return False
        return True

def main(args=None):
    rclpy.init(args=args)
    exploration_node = ExplorationNode()
    rclpy.spin(exploration_node)
    exploration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()