#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import websockets
import asyncio
import json
from threading import Thread
import time

class WebSocketIMUNode(Node):
    def __init__(self):
        super().__init__('websocket_imu_node')
        self.publisher_ = self.create_publisher(Imu, '/imu', 10)
        self.websocket_uri = "ws://10.147.19.226:5000/ws"
        
        # Состояние подключения
        self.is_connected = False
        self.connection_attempts = 0
        self.max_attempts = 5
        
        # Данные сенсоров
        self.latest_accel = None
        self.latest_gyro = None
        
        # Статистика
        self.message_counter = 0
        self.last_message_time = None
        
        # Запуск WebSocket клиента
        self.thread = Thread(target=self.run_websocket_loop)
        self.thread.daemon = True
        self.thread.start()
        
        # Таймер для диагностики
        self.diagnostic_timer = self.create_timer(1.0, self.print_diagnostics)

    def print_diagnostics(self):
        self.get_logger().info(
            f"Status: {'Connected' if self.is_connected else 'Disconnected'} | "
            f"Messages: {self.message_counter} | "
            f"Last msg: {'Never' if not self.last_message_time else time.strftime('%H:%M:%S', time.localtime(self.last_message_time))} | "
            f"Accel: {'Yes' if self.latest_accel else 'No'} | "
            f"Gyro: {'Yes' if self.latest_gyro else 'No'}"
        )

    def run_websocket_loop(self):
        while rclpy.ok():
            try:
                asyncio.run(self.websocket_client())
            except Exception as e:
                self.get_logger().error(f"WebSocket loop error: {str(e)}", throttle_duration_sec=5)
            time.sleep(1)

    async def websocket_client(self):
        self.get_logger().info(f"Connecting to {self.websocket_uri}...")
        
        try:
            async with websockets.connect(
                self.websocket_uri,
                ping_interval=20,
                ping_timeout=20,
                close_timeout=10
            ) as websocket:
                self.is_connected = True
                self.connection_attempts = 0
                self.get_logger().info("WebSocket connected successfully!")
                
                while self.is_connected and rclpy.ok():
                    try:
                        message = await websocket.recv()
                        self.message_counter += 1
                        self.last_message_time = time.time()
                        self.process_message(message)
                        
                    except websockets.exceptions.ConnectionClosed:
                        self.get_logger().warn("Connection closed by server")
                        self.is_connected = False
                        break
                    except Exception as e:
                        self.get_logger().error(f"Receive error: {str(e)}")
                        self.is_connected = False
                        break
                        
        except Exception as e:
            self.connection_attempts += 1
            self.get_logger().error(f"Connection error (attempt {self.connection_attempts}/{self.max_attempts}): {str(e)}")
            if self.connection_attempts >= self.max_attempts:
                self.get_logger().fatal("Max connection attempts reached!")
        finally:
            self.is_connected = False

    def process_message(self, message):
        try:
            # Сначала проверим общую структуру сообщения
            self.get_logger().debug(f"Raw message: {message[:200]}...")
            
            data = json.loads(message)
            
            # Проверяем новый формат сообщения
            if "sensor" not in data or "data" not in data:
                self.get_logger().warn("Invalid message structure - missing 'sensor' or 'data' fields")
                return
                
            sensor_type = data["sensor"].lower()
            sensor_data = data["data"]
            
            if sensor_type == "gyroscope":
                self.process_gyro_data(sensor_data)
            elif sensor_type == "accelerometer":
                self.process_accel_data(sensor_data)
            else:
                self.get_logger().warn(f"Unknown sensor type: {sensor_type}")
                return
                
            # Публикуем IMU если есть данные обоих сенсоров
            if self.latest_accel and self.latest_gyro:
                self.publish_imu()
                
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON: {message[:200]}...")
        except KeyError as e:
            self.get_logger().error(f"Missing field in message: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"Message processing error: {str(e)}")

    def process_gyro_data(self, data):
        try:
            self.latest_gyro = {
                'x': float(data['x']),
                'y': float(data['y']),
                'z': float(data['z']),
                'timestamp': data['Timestamp']
            }
            self.get_logger().debug(f"Gyro updated: {self.latest_gyro}")
        except Exception as e:
            self.get_logger().error(f"Gyro processing error: {str(e)}")

    def process_accel_data(self, data):
        try:
            self.latest_accel = {
                'x': float(data['x']),
                'y': float(data['y']),
                'z': float(data['z']),
                'timestamp': data['Timestamp']
            }
            self.get_logger().debug(f"Accel updated: {self.latest_accel}")
        except Exception as e:
            self.get_logger().error(f"Accel processing error: {str(e)}")

    def publish_imu(self):
        try:
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            
            # Акселерометр
            imu_msg.linear_acceleration = Vector3()
            imu_msg.linear_acceleration.x = self.latest_accel['x']
            imu_msg.linear_acceleration.y = self.latest_accel['y']
            imu_msg.linear_acceleration.z = self.latest_accel['z']
            
            # Гироскоп
            imu_msg.angular_velocity = Vector3()
            imu_msg.angular_velocity.x = self.latest_gyro['x']
            imu_msg.angular_velocity.y = self.latest_gyro['y']
            imu_msg.angular_velocity.z = self.latest_gyro['z']
            
            # Ковариационные матрицы
            imu_msg.orientation_covariance[0] = -1
            imu_msg.linear_acceleration_covariance = [0.04] * 9
            imu_msg.angular_velocity_covariance = [0.02] * 9
            
            self.publisher_.publish(imu_msg)
            self.get_logger().debug("IMU message published")
            
        except Exception as e:
            self.get_logger().error(f"Publish error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = WebSocketIMUNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()