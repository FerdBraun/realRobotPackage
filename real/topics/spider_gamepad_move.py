#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String
import evdev
from evdev import InputDevice, ecodes

class GamepadController:
    def __init__(self):
        self.node = rclpy.create_node('xbox_gamepad_publisher')
        self.publisher = self.node.create_publisher(String, '/spider_robot/command', 10)
        
        # Настройки для стика
        self.stick_deadzone = 10000  # Мертвая зона для предотвращения "дребезга"
        self.last_stick_command = None

        # Подключение геймпада
        self.gamepad = self.find_xbox_controller()
        if not self.gamepad:
            self.node.get_logger().error("Xbox controller not found!")
            raise RuntimeError("Gamepad not connected")
        self.node.get_logger().info(f"Connected to: {self.gamepad.name}")

    def find_xbox_controller(self):
        """Поиск Xbox контроллера"""
        devices = [InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            if "Xbox" in device.name or "X-Box" in device.name:
                return device
        try:
            return InputDevice('/dev/input/js0')  # Fallback
        except:
            return None

    def process_gamepad_events(self):
        """Основной цикл обработки событий"""
        self.node.get_logger().info("Listening for controller events...")
        try:
            for event in self.gamepad.read_loop():
                self.handle_event(event)
        except Exception as e:
            self.node.get_logger().error(f"Error: {str(e)}")

    def handle_event(self, event):
        """Обработка событий с учетом аналогового стика"""
        msg = String()
        command = None

        # Обработка кнопок
        if event.type == ecodes.EV_KEY:
            if event.code == ecodes.BTN_A and event.value == 1:
                command = 'F'
            elif event.code == ecodes.BTN_B and event.value == 1:
                command = 'B'
            elif event.code == ecodes.BTN_X and event.value == 1:
                command = 'L'
            elif event.code == ecodes.BTN_Y and event.value == 1:
                command = 'R'
            elif event.code == ecodes.BTN_START and event.value == 1:
                command = 'S'

        # Обработка аналогового стика (Левый стик)
        elif event.type == ecodes.EV_ABS:
            if event.code == ecodes.ABS_X:  # Горизонталь
                if abs(event.value) > self.stick_deadzone:
                    command = 'R' if event.value > 0 else 'L'
            elif event.code == ecodes.ABS_Y:  # Вертикаль
                if abs(event.value) > self.stick_deadzone:
                    command = 'F' if event.value < 0 else 'B'  # Инвертируем ось Y

        # Отправка команды (без проверки на повтор)
        if command:
            msg.data = command
            self.publisher.publish(msg)
            self.node.get_logger().info(f'Command: {command}')

def main(args=None):
    rclpy.init(args=args)
    try:
        controller = GamepadController()
        controller.process_gamepad_events()
    except KeyboardInterrupt:
        pass
    finally:
        controller.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
