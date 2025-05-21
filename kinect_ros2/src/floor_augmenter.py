#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from sklearn.linear_model import RANSACRegressor

# Импорт quaternion_matrix для поворота
from tf_transformations import quaternion_matrix, euler_matrix
import math

class FloorAugmenter(Node):
    def __init__(self):
        super().__init__('floor_augmenter')
        
        # Параметры
        self.declare_parameter('floor_height', -0.16)
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('max_floor_dist', 0.1)
        
        # Определяем поворот для точек
        # В вашем launch файле: arguments=['0', '0', '0.07', '0', '0', '-1.5708', 'base_link', 'kinect_depth']
        # Это RPY: roll=0, pitch=0, yaw=-1.5708 (т.е. -90 градусов вокруг Z)
        # НО! Мы не хотим применять ту же трансформацию, что и static_transform_publisher.
        # Мы хотим привести фрейм kinect_depth к фрейму, где Z - это пол.
        # Если в kinect_depth Z - вперед, Y - вниз, X - вправо,
        # и мы хотим, чтобы Z в нашем алгоритме был вверх.
        # Это означает, что нам нужно:
        # 1. Сделать y отрицательным (поскольку он идет вниз, а нам нужен вверх)
        # 2. Поменять местами Y и Z (т.е. Y станет Z, а Z станет Y)
        # Простая матрица поворота, которая меняет оси:
        # Новая X = Старая X
        # Новая Y = Старая Z
        # Новая Z = - Старая Y (чтобы Y вверх был положительным)
        # Это соответствует повороту на -90 градусов вокруг X, затем на 90 градусов вокруг Y,
        # или другим комбинациям.
        # Давайте попробуем простой обмен осей, который часто работает для Kinect:
        # Предположим, что для RANSAC нам нужно, чтобы ось 'высоты' была третьей координатой.
        # Оригинальные данные: (x, y, z) где z - глубина, y - вертикальная ось вниз.
        # Мы хотим: (x, -y, z_as_new_y), где -y - высота.
        # Или, более классический вариант для RANSAC пола: (x_new, y_new, z_new_height).
        # Если Kinect: X-вправо, Y-вниз, Z-вперед.
        # Для алгоритма: Z-вверх, X-вперед, Y-влево.
        # Тогда:
        # new_x = Z_kinect
        # new_y = -X_kinect (или X_kinect, в зависимости от желаемой ориентации)
        # new_z = -Y_kinect (высота, инвертируем, т.к. Y_kinect идет вниз)

        # Давайте попробуем такую перестановку:
        # Исходные точки (x, y, z) из Kinect
        # (x_kinect, y_kinect, z_kinect)
        # Хотим получить:
        # points_for_ransac[..., 0] = x_kinect
        # points_for_ransac[..., 1] = z_kinect (чтобы z_kinect стал новой Y для RANSAC X, Y)
        # points_for_ransac[..., 2] = -y_kinect (чтобы -y_kinect стал новой Z для RANSAC (высотой))

        # Вычисляем углы для поворота, если бы мы хотели использовать матрицу поворота.
        # Для простоты, давайте просто переставим и инвертируем оси.

        self.subscription = self.create_subscription(
            PointCloud2,
            'input_cloud',
            self.cloud_callback,
            10)
        
        self.publisher = self.create_publisher(
            PointCloud2,
            'output_cloud',
            10)
        
        self.get_logger().info("Floor augmenter node initialized")

    def detect_floor_plane(self, points_array_transformed):
        """Определение плоскости пола с помощью RANSAC
           points_array_transformed: массив (N, 3), где 2-я колонка (индекс 2) - это предполагаемая высота Z.
        """
        try:
            # Выбираем кандидатов в пол (ниже определенного Z)
            # Теперь points_array_transformed[:, 2] - это наша "высота Z"
            candidate_mask = points_array_transformed[:, 2] < (self.get_parameter('floor_height').value + 0.1)
            
            num_candidates = np.sum(candidate_mask)
            if num_candidates < 100:
                self.get_logger().warn(f"Not enough floor candidates for RANSAC ({num_candidates} found). Skipping floor detection.")
                return None
                
            floor_candidates = points_array_transformed[candidate_mask]
            
            if floor_candidates.shape[0] < 3: # RANSAC требует минимум 3 точки для плоскости
                self.get_logger().warn("Less than 3 floor candidates after filtering. Skipping RANSAC.")
                return None

            # RANSAC для поиска плоскости (ax + by + cz + d = 0)
            # X и Y для RANSAC берутся из первых двух столбцов transformed_points
            ransac = RANSACRegressor()
            X = floor_candidates[:, :2]  # x, y из transformed_points
            y = floor_candidates[:, 2]   # z (высота) из transformed_points
            ransac.fit(X, y)
            
            return ransac
        except Exception as e:
            self.get_logger().warn(f"Floor detection failed: {str(e)}")
            return None

    def generate_synthetic_floor(self, points_array_transformed, plane_model=None):
        """Генерация синтетических точек пола
           points_array_transformed: массив (N, 3), где 2-я колонка (индекс 2) - это предполагаемая высота Z.
        """
        
        # Границы облака (используем transformed_points)
        x_min, x_max = np.min(points_array_transformed[:, 0]), np.max(points_array_transformed[:, 0])
        y_min, y_max = np.min(points_array_transformed[:, 1]), np.max(points_array_transformed[:, 1])
        
        # Сетка точек
        voxel_size = self.get_parameter('voxel_size').value
        x = np.arange(x_min - voxel_size, x_max + voxel_size, voxel_size)
        y = np.arange(y_min - voxel_size, y_max + voxel_size, voxel_size)
        xx, yy = np.meshgrid(x, y)
        
        # Высота пола (используем transformed_points для предсказания)
        if plane_model:
            zz = plane_model.predict(np.vstack([xx.ravel(), yy.ravel()]).T)
        else:
            zz = np.full_like(xx, self.get_parameter('floor_height').value)
        
        # Возвращаем синтетический пол в transformed_frame
        return np.vstack([xx.ravel(), yy.ravel(), zz.ravel()]).T

    def cloud_callback(self, msg):
        """Обработка входящего облака точек"""
        try:
            cloud_data = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            
            if isinstance(cloud_data, np.ndarray):
                if cloud_data.dtype.names and all(f in cloud_data.dtype.names for f in ['x', 'y', 'z']):
                    points_array_raw = np.column_stack([cloud_data['x'], cloud_data['y'], cloud_data['z']]).astype(np.float32)
                else:
                    points_array_raw = np.asarray(cloud_data, dtype=np.float32)
                    if points_array_raw.ndim == 1:
                        if points_array_raw.size % 3 == 0:
                            points_array_raw = points_array_raw.reshape(-1, 3)
                        else:
                            self.get_logger().error(f"Received 1D array of size {points_array_raw.size} which is not divisible by 3. Cannot reshape.")
                            return
            else: # If cloud_data is an iterator
                points_list = list(cloud_data)
                if not points_list:
                    self.get_logger().warn("Received empty point cloud or iterator, skipping processing.")
                    return
                points_array_raw = np.array(points_list, dtype=np.float32)

            if points_array_raw.shape[0] == 0:
                self.get_logger().warn("Point cloud is empty after NaN filtering, skipping processing.")
                return
            
            if points_array_raw.ndim != 2 or points_array_raw.shape[1] != 3:
                self.get_logger().error(f"Point cloud array has incorrect shape: {points_array_raw.shape}. Expected (N, 3).")
                return

            # --- ГЛАВНОЕ ИЗМЕНЕНИЕ: ПРЕОБРАЗОВАНИЕ СИСТЕМЫ КООРДИНАТ ---
            # Предположим, что в оригинальном фрейме (kinect_depth):
            # X_orig = вправо от камеры
            # Y_orig = вниз от камеры
            # Z_orig = вперед от камеры (глубина)
            
            # Нам нужно, чтобы для алгоритма пола:
            # X_algo = X_orig (или Y_orig, или Z_orig, нужно выбрать 2 горизонтальные оси)
            # Y_algo = Y_orig (или Z_orig, нужно выбрать 2 горизонтальные оси)
            # Z_algo = ВВЕРХ (высота от пола)
            
            # Для Kinect часто удобна следующая перестановка/инверсия:
            # points_array_transformed[:, 0] = points_array_raw[:, 0]  # X остается X
            # points_array_transformed[:, 1] = points_array_raw[:, 2]  # Z (глубина) становится Y
            # points_array_transformed[:, 2] = -points_array_raw[:, 1] # -Y (вверх) становится Z (высота)
            #
            # Эта трансформация соответствует повороту, где Z-ось Kinect (глубина) становится Y-осью,
            # а Y-ось Kinect (вниз) становится -Z-осью (вверх).
            #
            # Проверяем, какой именно фрейм у вас на выходе из kinect_ros2_node.
            # Если Z_kinect_depth - это глубина, Y_kinect_depth - вертикаль вниз.
            # Нам нужно, чтобы "вертикаль вверх" стала Z для RANSAC.
            # Поэтому, points_array_transformed[:, 2] = -points_array_raw[:, 1] (инвертируем Y)
            # А две другие оси могут быть X и Z_kinect_depth.
            
            points_array_transformed = np.empty_like(points_array_raw)
            points_array_transformed[:, 0] = points_array_raw[:, 0] # X (горизонтальная)
            points_array_transformed[:, 1] = points_array_raw[:, 2] # Z (глубина, теперь горизонтальная)
            points_array_transformed[:, 2] = -points_array_raw[:, 1] # -Y (вертикальная вверх, теперь Z)

            # --- КОНЕЦ ПРЕОБРАЗОВАНИЯ СИСТЕМЫ КООРДИНАТ ---

            # Поиск плоскости пола (используем трансформированные точки)
            plane_model = self.detect_floor_plane(points_array_transformed)
            
            # Генерация синтетического пола (используем трансформированные точки)
            synthetic_floor_transformed = self.generate_synthetic_floor(points_array_transformed, plane_model)
            
            # Объединение облаков в трансформированной системе координат
            # Затем, нам нужно ПЕРЕВЕРНУТЬ ОБРАТНО для публикации в ROS
            
            # Объединяем исходные трансформированные точки и синтетические
            combined_transformed = np.vstack([points_array_transformed, synthetic_floor_transformed])
            
            # --- ОБРАТНОЕ ПРЕОБРАЗОВАНИЕ СИСТЕМЫ КООРДИНАТ ДЛЯ ПУБЛИКАЦИИ ---
            # new_x = X_kinect
            # new_y = -Z_algo (инвертируем Z_algo, чтобы получить Y_kinect_down)
            # new_z = Y_algo (чтобы получить Z_kinect_forward)
            
            combined_original_frame = np.empty_like(combined_transformed)
            combined_original_frame[:, 0] = combined_transformed[:, 0] # X_kinect_final = X_algo
            combined_original_frame[:, 1] = -combined_transformed[:, 2] # Y_kinect_final = -Z_algo (Y в Kinect вниз)
            combined_original_frame[:, 2] = combined_transformed[:, 1] # Z_kinect_final = Y_algo (Z в Kinect вперед)
            # --- КОНЕЦ ОБРАТНОГО ПРЕОБРАЗОВАНИЯ ---

            # Публикация нового облака (в оригинальном фрейме сообщения)
            new_cloud = point_cloud2.create_cloud_xyz32(msg.header, combined_original_frame)
            self.publisher.publish(new_cloud)
            
        except Exception as e:
            self.get_logger().error(f"Error processing cloud: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = FloorAugmenter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()