#!/usr/bin/env python3
import asyncio
import websockets
import sys
import tty
import termios
import json

# WebSocket server address
WS_SERVER = "ws://localhost:8765"  # Change this to your WebSocket server address

async def send_command(command):
    try:
        async with websockets.connect(WS_SERVER) as websocket:
            # Создаем JSON сообщение с командой
            message = json.dumps({"cmd": command})
            await websocket.send(message)
            print(f"Sent command: {command}")
    except Exception as e:
        print(f"Error sending command: {e}")

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    print("Keyboard control started. Press keys to send commands:")
    print("W - Step forward (F)")
    print("A - Turn left (L)")
    print("D - Turn right (R)")
    print("S - Reset position (S)")
    print("Q - Quit")
    
    try:
        while True:
            key = getch().lower()
            if key == 'w':
                asyncio.get_event_loop().run_until_complete(send_command('F'))
            elif key == 'a':
                asyncio.get_event_loop().run_until_complete(send_command('L'))
            elif key == 'd':
                asyncio.get_event_loop().run_until_complete(send_command('R'))
            elif key == 's':
                asyncio.get_event_loop().run_until_complete(send_command('B'))
            elif key == 'r':
                asyncio.get_event_loop().run_until_complete(send_command('S'))
            elif key == 'q':
                break
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
