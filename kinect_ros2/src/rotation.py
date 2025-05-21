#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from scipy.spatial.transform import Rotation as R
import struct
import time
from rclpy.qos import QoSProfile
class PointCloudRotator(Node):
    def __init__(self):
        super().__init__('point_cloud_rotator')
        import rclpy.qos

        qos_profile = rclpy.qos.QoSProfile(
            depth=50,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            deadline=rclpy.duration.Duration(seconds=0.1),
            lifespan=rclpy.duration.Duration(seconds=30)
        )


        self.subscription = self.create_subscription(
            PointCloud2,
            '/kinect/points',
            self.listener_callback,
            qos_profile=qos_profile)
    
        self.publisher = self.create_publisher(
            PointCloud2,
            '/kinect/rotated_points',
            qos_profile=qos_profile)
        
        self.get_logger().info('PointCloudRotator node initialized')

    def pointcloud2_to_array(self, cloud_msg):
        fields = cloud_msg.fields
        points = []
        for field in fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset

        fmt = '<' + 'f' * (len(fields))
        for data in struct.iter_unpack(fmt, cloud_msg.data):
            x = data[x_offset // 4]
            y = data[y_offset // 4]
            z = data[z_offset // 4]
            points.append([x, y, z])

        return np.array(points)

    def array_to_pointcloud2(self, points, stamp, frame_id):
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = stamp
        cloud_msg.header.frame_id = frame_id
        cloud_msg.height = 1
        cloud_msg.width = points.shape[0]
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        cloud_msg.data = points.astype(np.float32).tobytes()

        return cloud_msg

    def rotate_point_cloud(self, cloud, rotation_matrix):
        points = self.pointcloud2_to_array(cloud)
        rotated_points = np.dot(points, rotation_matrix.T)
        rotated_cloud = self.array_to_pointcloud2(rotated_points, cloud.header.stamp, cloud.header.frame_id)
        return rotated_cloud

    def listener_callback(self, msg):
        self.get_logger().info('Received point cloud message')
        
        r = R.from_euler('z', 90, degrees=True)
        rotation_matrix = r.as_matrix()

        rotated_cloud = self.rotate_point_cloud(msg, rotation_matrix)
        self.publisher.publish(rotated_cloud)
        self.get_logger().info('Published rotated point cloud')

def main(args=None):
    rclpy.init(args=args)
    point_cloud_rotator = PointCloudRotator()
    rclpy.spin(point_cloud_rotator)
    point_cloud_rotator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
