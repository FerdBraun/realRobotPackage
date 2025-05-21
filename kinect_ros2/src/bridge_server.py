#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import socket
import struct
import zlib
import numpy as np
import msgpack


class PointCloudBridgeServer(Node):
    def __init__(self):
        super().__init__('pointcloud_bridge_server')

        # Подписка на PointCloud2
        self.subscription = self.create_subscription(
            PointCloud2,
            '/kinect/points',
            self.pointcloud_callback,
            10
        )

        # TCP-сокет
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', 12345))
        self.server_socket.listen(1)
        self.conn, _ = self.server_socket.accept()
        self.get_logger().info("Client connected")

        # Кэширование заголовка
        self.last_header = None
        self.compression_level = 1

    def pack_pointcloud(self, msg):
        # Конвертация данных в numpy массив
        pc_data = np.frombuffer(msg.data, dtype=np.uint8)

        data = {
            'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
            'data': zlib.compress(pc_data.tobytes(), level=self.compression_level),
        }

        current_header = (
            msg.header.frame_id,
            msg.height,
            msg.width,
            msg.point_step,
            msg.row_step,
            msg.is_dense,
            msg.is_bigendian
        )

        if self.last_header != current_header:
            fields = [{'name': f.name, 'offset': f.offset, 'datatype': f.datatype, 'count': f.count} for f in msg.fields]
            data['header'] = {
                'frame_id': msg.header.frame_id,
                'height': msg.height,
                'width': msg.width,
                'point_step': msg.point_step,
                'row_step': msg.row_step,
                'is_dense': msg.is_dense,
                'is_bigendian': msg.is_bigendian,
                'fields': fields
            }
            self.last_header = current_header

        return data

    def pointcloud_callback(self, msg):
        try:
            packed_data = self.pack_pointcloud(msg)
            serialized = msgpack.packb(packed_data, use_bin_type=True)

            # Отправляем длину + данные
            self.conn.sendall(struct.pack('>I', len(serialized)))
            self.conn.sendall(serialized)

        except Exception as e:
            self.get_logger().error(f"Ошибка передачи: {e}")
            self.conn.close()
            self.server_socket.close()
            raise


def main(args=None):
    rclpy.init(args=args)
    server = PointCloudBridgeServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
