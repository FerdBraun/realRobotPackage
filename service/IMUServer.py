import asyncio
import websockets
import socket
import json
from collections import defaultdict
import numpy as np
from pykalman import KalmanFilter
from typing import Dict, Set, Any, DefaultDict, List, Optional

class SensorProcessor:
    def __init__(self):
        # Инициализация фильтров Калмана
        self.kalman_filters = {
            'accelerometer': {
                'x': self._init_kalman(),
                'y': self._init_kalman(),
                'z': self._init_kalman()
            },
            'gyroscope': {
                'x': self._init_kalman(),
                'y': self._init_kalman(),
                'z': self._init_kalman()
            },
            'orientation': {
                'azimuth': self._init_kalman(),
                'pitch': self._init_kalman(),
                'roll': self._init_kalman()
            }
        }
        
        # Калибровочные параметры
        self.calibration = {
            'accelerometer': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'gyroscope': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'orientation': {'azimuth': 0.0, 'pitch': 0.0, 'roll': 0.0},
            'calibrated': False
        }
        
        # Подключенные клиенты
        self.connected_clients: Set[websockets.WebSocketServerProtocol] = set()
        
        # Буфер данных для калибровки
        self.calibration_buffer: DefaultDict[str, List[float]] = defaultdict(list)
        
        # Параметры калибровки
        self.calibration_samples_required = 100
    
    def _init_kalman(self) -> KalmanFilter:
        """Инициализация фильтра Калмана"""
        return KalmanFilter(
            transition_matrices=1,
            observation_matrices=1,
            initial_state_mean=0,
            initial_state_covariance=1,
            observation_covariance=0.1,
            transition_covariance=0.01
        )
    
    def apply_kalman(self, sensor_type: str, axis: str, value: float) -> float:
        """Применение фильтра Калмана"""
        try:
            kf = self.kalman_filters[sensor_type][axis]
            filtered_state, _ = kf.filter_update(
                kf.initial_state_mean,
                kf.initial_state_covariance,
                observation=value
            )
            return float(filtered_state[0])
        except KeyError:
            return value
    
    def calibrate_sensors(self, sensor_type: str, data: Dict[str, float]):
        """Калибровка датчиков"""
        if sensor_type not in self.calibration:
            return
            
        for axis, value in data.items():
            self.calibration_buffer[f"{sensor_type}_{axis}"].append(value)
        
        if (not self.calibration['calibrated'] and 
            all(len(v) >= self.calibration_samples_required 
                for k, v in self.calibration_buffer.items() 
                if k.startswith(sensor_type))):
            
            for axis in data.keys():
                key = f"{sensor_type}_{axis}"
                self.calibration[sensor_type][axis] = np.mean(self.calibration_buffer[key])
                del self.calibration_buffer[key]
            
            print(f"{sensor_type.capitalize()} calibration completed")
            
            if all(len(v) == 0 for v in self.calibration_buffer.values()):
                self.calibration['calibrated'] = True
                print("All sensors calibrated!")
    
    def apply_calibration(self, sensor_type: str, data: Dict[str, float]) -> Dict[str, float]:
        """Применение калибровки"""
        if sensor_type not in self.calibration or not self.calibration['calibrated']:
            return data
        
        return {
            axis: value - self.calibration[sensor_type][axis]
            for axis, value in data.items()
        }
    
    async def broadcast_data(self, sensor_type: str, data: Dict[str, float]):
        """Отправка данных клиентам"""
        if not self.connected_clients:
            return
            
        message = json.dumps({
            'sensor': sensor_type,
            'data': data,
            'timestamp': asyncio.get_event_loop().time(),
            'calibrated': self.calibration['calibrated']
        })
        
        tasks = []
        for client in self.connected_clients:
            try:
                tasks.append(client.send(message))
            except:
                continue
                
        await asyncio.gather(*tasks, return_exceptions=True)

class SensorServer:
    def __init__(self):
        self.processor = SensorProcessor()
        self.ip_addr = self.get_ip()
        self.port = 5000
        self.log_to_file = False
        
        print(f"\nIMU Server running on ws://{self.ip_addr}:{self.port}")
        print("Available endpoints:")
        print("- /ws - для получения данных")
        print("- /accelerometer - для отправки данных акселерометра")
        print("- /gyroscope - для отправки данных гироскопа")
        print("- /orientation - для отправки данных ориентации\n")
    
    @staticmethod
    def get_ip() -> str:
        """Получение IP адреса сервера"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(('10.255.255.255', 1))
                return s.getsockname()[0]
        except:
            return '127.0.0.1'
    
    def parse_sensor_data(self, sensor_type: str, raw_data: Dict) -> Optional[Dict[str, float]]:
        """Парсинг входящих данных"""
        try:
            if sensor_type == 'orientation':
                # Поддержка двух форматов данных ориентации
                if 'SensorName' in raw_data:  # Старый формат
                    return {
                        'azimuth': float(raw_data.get('azimuth', 0)),
                        'pitch': float(raw_data.get('pitch', 0)),
                        'roll': float(raw_data.get('roll', 0))
                    }
                elif 'data' in raw_data:  # Новый формат
                    return {
                        'azimuth': float(raw_data['data'].get('azimuth', 0)),
                        'pitch': float(raw_data['data'].get('pitch', 0)),
                        'roll': float(raw_data['data'].get('roll', 0))
                    }
            else:
                # Обработка акселерометра и гироскопа
                if 'data' in raw_data:  # Новый формат
                    data = raw_data['data']
                else:  # Старый формат
                    data = raw_data
                
                if all(axis in data for axis in ['x', 'y', 'z']):
                    return {
                        'x': float(data['x']),
                        'y': float(data['y']),
                        'z': float(data['z'])
                    }
        
        except (ValueError, TypeError) as e:
            print(f"Error parsing {sensor_type} data: {e}")
        
        return None
    
    async def handle_sensor_data(self, websocket: websockets.WebSocketServerProtocol, path: str):
        """Обработка входящих данных"""
        sensor_type = path.lstrip('/')
        
        async for message in websocket:
            try:
                raw_data = json.loads(message)
                sensor_data = self.parse_sensor_data(sensor_type, raw_data)
                
                if sensor_data is None:
                    print(f"Invalid {sensor_type} data format: {raw_data}")
                    continue
                
                # Калибровка
                if not self.processor.calibration['calibrated']:
                    self.processor.calibrate_sensors(sensor_type, sensor_data)
                
                # Применение калибровки
                calibrated_data = self.processor.apply_calibration(sensor_type, sensor_data)
                
                # Фильтрация
                filtered_data = {
                    axis: self.processor.apply_kalman(sensor_type, axis, value)
                    for axis, value in calibrated_data.items()
                }
                
                # Логирование
                if self.log_to_file:
                    with open(f"{sensor_type}_log.txt", "a") as f:
                        f.write(json.dumps({
                            'raw': sensor_data,
                            'calibrated': calibrated_data,
                            'filtered': filtered_data,
                            'timestamp': asyncio.get_event_loop().time()
                        }) + "\n")
                
                # Отправка данных
                await self.processor.broadcast_data(sensor_type, filtered_data)
                print(f"{sensor_type}: {filtered_data}")
            
            except json.JSONDecodeError:
                print(f"Invalid JSON received: {message[:100]}...")
            except Exception as e:
                print(f"Error processing {sensor_type} data: {e}")
    
    async def websocket_handler(self, websocket: websockets.WebSocketServerProtocol, path: str):
        """Основной обработчик соединений"""
        try:
            if path == '/ws':
                # Клиент для получения данных
                self.processor.connected_clients.add(websocket)
                print(f"New client connected (total: {len(self.processor.connected_clients)})")
                
                try:
                    await websocket.wait_closed()
                finally:
                    self.processor.connected_clients.discard(websocket)
                    print(f"Client disconnected (remaining: {len(self.processor.connected_clients)})")
            else:
                # Обработка данных датчиков
                await self.handle_sensor_data(websocket, path)
        except Exception as e:
            print(f"WebSocket error: {e}")
    
    async def run(self):
        """Запуск сервера"""
        async with websockets.serve(
            self.websocket_handler,
            '0.0.0.0',
            self.port,
            ping_interval=20,
            ping_timeout=20,
            max_size=1_000_000
        ):
            print(f"Server started on ws://{self.ip_addr}:{self.port}")
            await asyncio.Future()  # Бесконечное ожидание

if __name__ == "__main__":
    server = SensorServer()
    try:
        asyncio.run(server.run())
    except KeyboardInterrupt:
        print("\nServer stopped by user")
