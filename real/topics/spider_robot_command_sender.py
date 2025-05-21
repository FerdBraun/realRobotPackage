#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String
from pynput import keyboard

def on_press(key):
    try:
        if key.char == 'd':
            msg = String()
            msg.data = 'R'
            publisher.publish(msg)
        elif key.char == 'a':
            msg = String()
            msg.data = 'L'
            publisher.publish(msg)
        elif key.char == 'w':
            msg = String()
            msg.data = 'F'
            publisher.publish(msg)
        elif key.char == 's':
            msg = String()
            msg.data = 'B'
            publisher.publish(msg)
    except AttributeError:
        pass

def main(args=None):
    global publisher
    rclpy.init(args=args)
    node = rclpy.create_node('keyboard_publisher')
    publisher = node.create_publisher(String, '/spider_robot/command', 10)
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
