#!/usr/bin/env python3
import websockets
import asyncio
import json
from SpiderRobot import SpiderRobot

class MotionExecutor:
    def __init__(self):
        self.robot = SpiderRobot()
        self.command_map = {
            'F': self.robot.step_forward,
            'L': self.robot.turn_left,
            'R': self.robot.turn_right,
            'B': self.robot.step_backward,
            'S': self.robot.reset_position
        }
        
    async def handle_command(self, websocket):
        async for message in websocket:
            print(websocket)
            try:
                data = json.loads(message)
                cmd = data.get('cmd')
                if cmd in self.command_map:
                    print(f"Executing command: {cmd}")
                    self.command_map[cmd]()
                else:
                    print(f"Unknown command: {cmd}")
            except json.JSONDecodeError:
                print("Invalid JSON received")
            except Exception as e:
                print(f"Error processing command: {e}")

async def main():
    executor = MotionExecutor()
    executor.command_map['S']() # self.command_map['S']()
    async with websockets.serve(executor.handle_command, "0.0.0.0", 8765):
        print("Motion executor ready on ws://0.0.0.0:8765")
        await asyncio.Future()  # Run forever

if __name__ == '__main__':
    asyncio.run(main())