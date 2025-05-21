#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import websockets
import asyncio
import json
from std_msgs.msg import String

import threading

class SpiderRobotCommandNode(Node):
    def __init__(self):
        super().__init__('spider_robot_command_node')
        self.subscription = self.create_subscription(
            String,
            '/spider_robot/command',
            self.listener_callback,
            10)
        self.websocket_uri = "ws://10.147.19.226:8765"
        self.current_task = None
        self.loop = asyncio.new_event_loop()  # Создаем новый event loop
        self.thread = threading.Thread(target=self.start_loop, daemon=True)
        self.thread.start()  # Запускаем loop в отдельном потоке
        self.get_logger().info("Spider Robot Command Node started")

    def start_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def send_websocket_command(self, cmd):
        try:
            async with websockets.connect(self.websocket_uri, max_queue=1) as websocket:
                command = json.dumps({'cmd': cmd})
                await websocket.send(command)
                self.get_logger().info(f"Sent command: {cmd}")
        except Exception as e:
            self.get_logger().error(f"WebSocket error: {str(e)}")

    def listener_callback(self, msg):
        cmd = msg.data.strip().upper()
        valid_commands = ['F', 'L', 'R', 'S']
        
        if cmd in valid_commands:
            self.get_logger().info(f"Received valid command: {cmd}")
            
            if self.current_task and not self.current_task.done():
                self.current_task.cancel()
                self.get_logger().info("Cancelled previous command")
            
            # Запускаем задачу в отдельном потоке через loop.call_soon_threadsafe
            self.current_task = asyncio.run_coroutine_threadsafe(
                self.send_websocket_command(cmd),
                self.loop
            )
        else:
            self.get_logger().warn(f"Received invalid command: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = SpiderRobotCommandNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.current_task and not node.current_task.done():
            node.current_task.cancel()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
