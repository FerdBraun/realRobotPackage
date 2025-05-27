#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String
import evdev
from evdev import InputDevice, ecodes, categorize

class GamepadController:
    def __init__(self):
        self.node = rclpy.create_node('xbox_gamepad_publisher')
        self.publisher = self.node.create_publisher(String, '/spider_robot/command', 10)
        self.last_command = None

        # Настройки для Xbox Series Controller
        self.gamepad = self.find_xbox_controller()
        if not self.gamepad:
            self.node.get_logger().error("Xbox controller not found!")
            raise RuntimeError("Gamepad not connected")

        self.node.get_logger().info(f"Connected to: {self.gamepad.name}")

    def find_xbox_controller(self):
        """Ищет конкретно Xbox контроллер"""
        devices = [InputDevice(path) for path in evdev.list_devices()]
        xbox_names = [
            "Xbox Controller",
            "X-Box Controller",
            "Xbox Series",
            "Microsoft X-Box"
        ]
        
        for device in devices:
            self.node.get_logger().info(f"Checking device: {device.name}")
            if any(name in device.name for name in xbox_names):
                self.node.get_logger().info("Xbox controller detected!")
                return device
        
        # Если не нашли по имени, попробуем через /dev/input/js0
        try:
            js_device = InputDevice('/dev/input/js0')
            self.node.get_logger().info("Using /dev/input/js0 as fallback")
            return js_device
        except:
            return None

    def process_gamepad_events(self):
        """Основной цикл обработки событий"""
        self.node.get_logger().info("Listening for Xbox controller events...")
        try:
            for event in self.gamepad.read_loop():
                self.handle_event(event)
        except OSError as e:
            self.node.get_logger().error(f"Controller error: {str(e)}")

    def handle_event(self, event):
        """Обработка событий Xbox контроллера"""
        msg = String()
        command = None

        # Кнопки Xbox Series (проверьте свои коды при необходимости)
        if event.type == ecodes.EV_KEY:
            if event.code == ecodes.BTN_A and event.value == 1:      # Кнопка A (зеленая)
                command = 'F'  # Вперед
            elif event.code == ecodes.BTN_B and event.value == 1:    # Кнопка B (красная)
                command = 'B'  # Назад
            elif event.code == ecodes.BTN_X and event.value == 1:    # Кнопка X (синяя)
                command = 'L'  # Влево
            elif event.code == ecodes.BTN_Y and event.value == 1:    # Кнопка Y (желтая)
                command = 'R'  # Вправо
            elif event.code == ecodes.BTN_START and event.value == 1: # Кнопка Start
                command = 'S'  # Стоп

        # Обработка крестовины (D-Pad)
        elif event.type == ecodes.EV_ABS:
            if event.code == ecodes.ABS_HAT0X:
                if event.value == 1:    # Вправо
                    command = 'R'
                elif event.value == -1:  # Влево
                    command = 'L'
            elif event.code == ecodes.ABS_HAT0Y:
                if event.value == 1:    # Вниз
                    command = 'B'
                elif event.value == -1:  # Вверх
                    command = 'F'

        # Аналоговые стики (если нужно)
        # elif event.type == ecodes.EV_ABS:
        #     if event.code == ecodes.ABS_X:  # Левый стик X
        #         if event.value > 20000:
        #             command = 'R'
        #         elif event.value < -20000:
        #             command = 'L'
        #     elif event.code == ecodes.ABS_Y:  # Левый стик Y
        #         if event.value > 20000:
        #             command = 'B'
        #         elif event.value < -20000:
        #             command = 'F'

        if command:
            msg.data = command
            self.publisher.publish(msg)
            self.last_command = command
            self.node.get_logger().info(f'Command: {command}')

def main(args=None):
    rclpy.init(args=args)
    try:
        controller = GamepadController()
        controller.process_gamepad_events()
    except Exception as e:
        controller.node.get_logger().error(f"Error: {str(e)}")
    finally:
        if 'controller' in locals():
            controller.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
