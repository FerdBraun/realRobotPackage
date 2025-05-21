#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class TopicRelay(Node):
    def __init__(self):
        super().__init__('topic_relay')
        
        # Создаем QoS профиль с volatile durability
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.SYSTEM_DEFAULT,  # или BEST_EFFORT
            durability=QoSDurabilityPolicy.SYSTEM_DEFAULT
        )
        
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/rtabmap/map',
            self.callback,
            qos_profile=qos_profile)  # используем тот же профиль для подписчика
        
        self.pub = self.create_publisher(
            OccupancyGrid, 
            '/map_nav2', 
            qos_profile=qos_profile)
            
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/map_nav2',
            self.callback,
            qos_profile=qos_profile)  # используем тот же профиль для подписчика
        
        self.pub = self.create_publisher(
            OccupancyGrid, 
            '/map', 
            qos_profile=qos_profile)

    def callback(self, msg):
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = TopicRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()