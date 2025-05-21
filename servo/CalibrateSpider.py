#!/usr/bin/env python3
from SpiderRobot import SpiderRobot

def main():
    print("=== Spider Robot Calibration ===")
    print("1. Interactive calibration")
    print("2. Simple calibration (set all to 90 degrees)")
    print("3. Exit")
    
    choice = input("Select option (1-3): ")
    
    spider = SpiderRobot()
    
    if choice == '1':
        spider.interactive_calibration()
    elif choice == '2':
        spider.simple_calibrate()
    elif choice == '3':
        print("Exiting...")
    else:
        print("Invalid choice")

if __name__ == "__main__":
    main()