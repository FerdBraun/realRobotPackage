#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PointStamped
import math
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from tf_transformations import euler_from_quaternion
import time

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # Подписка на путь
        self.subscription = self.create_subscription(
            Path,
            '/path',
            self.path_callback,
            1  # Уменьшите размер очереди до 1
        )

        # Публикация команд для робота
        self.cmd_pub = self.create_publisher(String, '/spider_robot/command', 10)

        # TF для получения ориентации робота
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Параметры
        self.command_delay = 0.5  # Секунды
        self.distance_threshold = 0.15
        self.angle_threshold_deg = 20  # Градусы

        self.path = []
        self.current_index = 1
        self.last_command_time = time.time()
        self.command_interval = 1.5 # Интервал между командами
        self.timer = None
        self.busy = False

    def path_callback(self, msg):
        """Обработчик пути"""
        self.get_logger().info("New path received. Resetting state...")

        # Если уже обрабатывается путь, завершаем его
        if self.busy:
            self.get_logger().warn("Already processing a path. Cancelling current path...")
            self.cancel_current_path()

        if not msg.poses:
            self.get_logger().warn("Received empty path")
            return

        # Логирование первых точек пути
        for i, pose in enumerate(msg.poses[:3]):
            x, y = pose.pose.position.x, pose.pose.position.y
            self.get_logger().info(f"Original path point {i}: X={x:.2f}, Y={y:.2f}")

        # Фильтрация слишком близких точек
        self.path = self.filter_close_points(msg.poses)
        if len(self.path) < 2:
            self.get_logger().warn("Path too short after filtering")
            return

        self.get_logger().info(f"Received path with {len(msg.poses)} poses, filtered to {len(self.path)} points")
        self.current_index = 1
        self.busy = True
        self.timer = self.create_timer(self.command_delay, self.timer_callback)

    def cancel_current_path(self):
        """Отмена текущего пути и сброс состояния"""
        if self.timer is not None:
            self.get_logger().info("Cancelling existing timer...")
            self.timer.cancel()
            self.timer = None
        self.path = []
        self.current_index = 1
        self.last_command_time = time.time()
        self.busy = False
        self.get_logger().info("Current path cancelled and state reset.")

    def filter_close_points(self, poses):
        """Фильтрация слишком близких точек"""
        filtered = [poses[0]]
        for pose in poses[1:]:
            last = filtered[-1]
            if self.distance(last.pose.position, pose.pose.position) > self.distance_threshold:
                filtered.append(pose)
        return filtered

    def timer_callback(self):
        """Обработчик таймера для следования по пути"""
        if not self.busy or not self.path:
            self.get_logger().warn("No active path or busy flag is False. Cancelling timer.")
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
            return

        if self.current_index >= len(self.path):
            self.get_logger().info("Path completed")
            self.cancel_current_path()
            return

        # Получаем текущую и следующую точки
        curr = self.path[self.current_index - 1].pose.position
        next = self.path[self.current_index].pose.position

        # Вектор направления (для робота вперед - это ось Y)
        dx = next.x - curr.x
        dy = next.y - curr.y

        # Угол к цели (atan2(dy, dx), так как вперед - ось Y)
        target_angle = math.atan2(-dx, dy)

        # Получаем текущую позицию и ориентацию робота
        current_position, current_angle = self.get_robot_position_and_orientation()
        if current_position is None:
            return

        # Рассчитываем разницу углов
        angle_diff = self.normalize_angle(target_angle - current_angle)
        angle_diff_deg = math.degrees(angle_diff)

        # Логирование информации
        self.get_logger().info(
            f"Point {self.current_index}/{len(self.path)-1}\n"
            f"Current: ({curr.x:.2f}, {curr.y:.2f})\n"
            f"Next: ({next.x:.2f}, {next.y:.2f})\n"
            f"Direction vector: X:{dx:.2f}, Y:{dy:.2f}\n"
            f"Target angle: {math.degrees(target_angle):.1f}°\n"
            f"Current angle: {math.degrees(current_angle):.1f}°\n"
            f"Angle diff: {angle_diff_deg:.1f}°"
        )

        # Проверяем временной интервал
        current_time = time.time()
        if current_time - self.last_command_time < self.command_interval:
            return

        # Определяем команду
        if abs(angle_diff_deg) > self.angle_threshold_deg:
            if angle_diff_deg > 0:
                self.send_command('L')  # Поворот влево
            else:
                self.send_command('R')  # Поворот вправо
        else:
            self.send_command('F')  # Движение вперед
            self.current_index += 1

        self.last_command_time = current_time

    def send_command(self, cmd):
        """Отправка команды роботу"""
        self.cmd_pub.publish(String(data=cmd))
        self.get_logger().info(f"Sent command: {cmd} at {time.time()}")

    def distance(self, p1, p2):
        """Расстояние между двумя точками"""
        return math.hypot(p2.x - p1.x, p2.y - p1.y)

    def normalize_angle(self, angle):
        """Нормализация угла в диапазоне от -π до π"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def get_robot_position_and_orientation(self):
        """Получение текущей позиции и ориентации робота"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            
            # Преобразуем кватернион в эйлеровы углы
            quaternion = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            euler_angles = euler_from_quaternion(quaternion)
            
            # Логирование информации о трансформации
            self.get_logger().info(
                f"Robot transform:\n"
                f"Position: X={transform.transform.translation.x:.2f}, Y={transform.transform.translation.y:.2f}\n"
                f"Orientation (yaw): {math.degrees(euler_angles[2]):.1f}°"
            )
            
            return transform.transform.translation, euler_angles[2]  # Возвращаем yaw
 
        except Exception as e:
            self.get_logger().error(f'TF error: {str(e)}')
            return None, 0.0

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()