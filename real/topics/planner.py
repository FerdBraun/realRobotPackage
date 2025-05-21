#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point, PoseStamped, PointStamped, Pose
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import math
import heapq
import time

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        # Подписки
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10
        )
        self.goal_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.goal_callback,
            10
        )
        # Публикатор для пути
        self.path_pub = self.create_publisher(
            Path,
            '/path',
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
        self.current_goal = None
        self.last_path = None
        self.buffer_distance = 0.1 # Буферная зона в метрах
        # Увеличенные интервалы перепланирования
        self.replan_interval = 5.0  # Основной интервал проверки (было 5.0)
        self.min_replan_interval = 5.0  # Минимальный интервал между перепланировками
        self.last_replan_time = time.time()
        # Таймер для перепланирования
        self.timer = self.create_timer(self.replan_interval, self.replan_callback)
        self.get_logger().info(f'A* Planner initialized with replan interval: {self.replan_interval} sec')

    def costmap_callback(self, msg):
        """Обработчик карты стоимости с проверкой изменений"""
        current_time = time.time()
        if self.costmap is None or self.costmap != msg:
            self.costmap = msg
            self.resolution = msg.info.resolution
            self.origin_x = msg.info.origin.position.x
            self.origin_y = msg.info.origin.position.y
            self.get_logger().info('Costmap updated')
            # Перепланируем только если с последней перепланировки прошло достаточно времени
            if (self.current_goal is not None and 
                (current_time - self.last_replan_time) > self.min_replan_interval):
                self.execute_planning(self.current_goal)

    def goal_callback(self, msg):
        """Обработчик целевой точки"""
        if self.costmap is None:
            self.get_logger().warn('Costmap not received yet')
            return
        self.current_goal = msg
        self.execute_planning(msg)

    def execute_planning(self, msg):
        """Основная логика планирования"""
        try:
            self.last_replan_time = time.time()
            robot_position = self.get_robot_position()
            map_start = self.transform_to_map(robot_position)
            map_goal = self.transform_to_map(msg)
            grid_start = self.world_to_grid(map_start.point.x, map_start.point.y)
            grid_goal = self.world_to_grid(map_goal.point.x, map_goal.point.y)
            self.get_logger().info(f'Planning from {grid_start} to {grid_goal}')
            path = self.a_star(grid_start, grid_goal)
            self.last_path = path
            if path:
                self.publish_path(path, map_goal.header)
            else:
                self.get_logger().warn('No path found')
                self.publish_path([], map_goal.header)
        except Exception as e:
            self.get_logger().error(f'Error in planning: {str(e)}', throttle_duration_sec=10)
            self.publish_path([], msg.header)

    def get_robot_position(self):
        """Получение текущей позиции робота в системе координат map"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            robot_position = PointStamped()
            robot_position.header.frame_id = 'base_link'
            robot_position.header.stamp = self.get_clock().now().to_msg()
            robot_position.point.x = 0.0
            robot_position.point.y = 0.0
            robot_position.point.z = 0.0
            return do_transform_point(robot_position, transform)
        except Exception as e:
            self.get_logger().error(f'Error in getting robot position: {str(e)}', throttle_duration_sec=10)
            raise

    def transform_to_map(self, point_stamped):
        """Преобразование точки в систему координат карты"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                point_stamped.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            return do_transform_point(point_stamped, transform)
        except Exception as e:
            self.get_logger().error(f'Transform error: {str(e)}', throttle_duration_sec=10)
            raise

    def world_to_grid(self, wx, wy):
        """Конвертация мировых координат в координаты сетки"""
        mx = int((wx - self.origin_x) / self.resolution)
        my = int((wy - self.origin_y) / self.resolution)
        return (mx, my)

    def grid_to_world(self, mx, my):
        """Конвертация координат сетки в мировые координаты"""
        wx = self.origin_x + mx * self.resolution
        wy = self.origin_y + my * self.resolution
        return (wx, wy)

    def a_star(self, start, goal):
        """Реализация алгоритма A*"""
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, current)
            for neighbor in self.get_neighbors(current):
                if not self.is_safe_cell(neighbor):
                    continue
                tentative_g = g_score[current] + self.get_cost(current, neighbor)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return None

    def heuristic(self, a, b):
        """Эвристическая функция (Евклидово расстояние)"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def get_neighbors(self, cell):
        """Получение соседних клеток (8-связность)"""
        x, y = cell
        neighbors = [
            (x+1, y), (x-1, y),
            (x, y+1), (x, y-1),
            (x+1, y+1), (x-1, y-1),
            (x+1, y-1), (x-1, y+1)
        ]
        return neighbors

    def is_safe_cell(self, cell):
        """Проверка безопасности клетки с учетом буферной зоны"""
        if self.costmap is None:
            return False
        x, y = cell
        buffer_cells = int(self.buffer_distance / self.resolution)
        for dy in range(-buffer_cells, buffer_cells + 1):
            for dx in range(-buffer_cells, buffer_cells + 1):
                nx, ny = x + dx, y + dy
                if nx < 0 or ny < 0 or nx >= self.costmap.info.width or ny >= self.costmap.info.height:
                    return False
                index = ny * self.costmap.info.width + nx
                if self.costmap.data[index] > 0:
                    return False
        return True

    def get_cost(self, from_cell, to_cell):
        """Получение стоимости перехода между клетками"""
        if from_cell[0] != to_cell[0] and from_cell[1] != to_cell[1]:
            return math.sqrt(2)
        return 1.0

    def reconstruct_path(self, came_from, current):
        """Восстановление пути"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def publish_path(self, path, header):
        """Публикация пути"""
        path_msg = Path()
        path_msg.header = header
        for grid_point in path:
            wx, wy = self.grid_to_world(*grid_point)
            pose = PoseStamped()
            pose.header = header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def is_goal_reached(self, current_pos, goal_pos):
        """Проверка достижения цели"""
        distance = math.sqrt((current_pos.point.x - goal_pos.point.x)**2 +
                          (current_pos.point.y - goal_pos.point.y)**2)
        return distance < 0.3  # Допуск 0.3 метра

    def replan_callback(self):
        """Периодическое перепланирование с увеличенными интервалами"""
        current_time = time.time()
        if self.current_goal is None:
            return
            
        # Проверяем минимальный интервал между перепланировками
        if (current_time - self.last_replan_time) < self.min_replan_interval:
            return
            
        try:
            robot_pos = self.get_robot_position()
            goal_map = self.transform_to_map(self.current_goal)
            
            if self.is_goal_reached(robot_pos, goal_map):
                self.get_logger().info("Goal reached!")
                self.current_goal = None
                self.publish_path([], goal_map.header)
                return
                
            # Проверяем только первые 10 точек пути для экономии ресурсов
            if self.last_path:
                for cell in self.last_path[:10]:
                    if not self.is_safe_cell(cell):
                        self.get_logger().info("Obstacle detected nearby, replanning...")
                        self.execute_planning(self.current_goal)
                        break
                        
        except Exception as e:
            self.get_logger().error(f"Replan error: {str(e)}", throttle_duration_sec=10)

def main(args=None):
    rclpy.init(args=args)
    planner = AStarPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()