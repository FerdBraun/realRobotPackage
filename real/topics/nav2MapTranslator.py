#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_map_transform_publisher')
        
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_transform()
        
        # Публикуем трансформацию каждые 5 секунд для надежности
        self.timer = self.create_timer(5.0, self.publish_transform)
    
    def publish_transform(self):
        transform = TransformStamped()
        
        # Заполняем заголовок трансформации
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map_nav2'       # Исходный фрейм
        transform.child_frame_id = 'map'   # Целевой фрейм
        
        # Задаем смещение (подберите нужные значения)
        transform.transform.translation.x = -5.0
        transform.transform.translation.y = -5.0
        transform.transform.translation.z = 0.0
        
        # Без вращения
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        self.tf_static_broadcaster.sendTransform(transform)
        self.get_logger().info('Publishing transform from map to map_nav2')

def main(args=None):
    rclpy.init(args=args)
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
