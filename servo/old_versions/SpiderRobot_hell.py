import time
import argparse
import json
import os
from adafruit_servokit import ServoKit

class SpiderRobot:
    def __init__(self):
        # Initialize PCA9685 drivers
        self.right_legs_kit = ServoKit(channels=16, address=0x40)
        self.left_legs_kit = ServoKit(channels=16, address=0x41)
        
        # Number of legs
        self.num_legs = 6
        
        # Servo specifications
        self.servo_min_pulse = 500
        self.servo_max_pulse = 2500
        self.servo_frequency = 50
        
        # Set PWM frequency
        self.right_legs_kit.frequency = self.servo_frequency
        self.left_legs_kit.frequency = self.servo_frequency
        
        # Legs configuration
        self.legs = [
            # Right legs (0x40)
            {'coxa': self.right_legs_kit.servo[15], 'femur': self.right_legs_kit.servo[14], 'tibia': self.right_legs_kit.servo[13]},  # Leg 0
            {'coxa': self.right_legs_kit.servo[12], 'femur': self.right_legs_kit.servo[11], 'tibia': self.right_legs_kit.servo[10]},  # Leg 1
            {'coxa': self.right_legs_kit.servo[9], 'femur': self.right_legs_kit.servo[8], 'tibia': self.right_legs_kit.servo[7]},      # Leg 2
            
            # Left legs (0x41)
            {'coxa': self.left_legs_kit.servo[0], 'femur': self.left_legs_kit.servo[1], 'tibia': self.left_legs_kit.servo[2]},        # Leg 3
            {'coxa': self.left_legs_kit.servo[3], 'femur': self.left_legs_kit.servo[4], 'tibia': self.left_legs_kit.servo[5]},        # Leg 4
            {'coxa': self.left_legs_kit.servo[6], 'femur': self.left_legs_kit.servo[7], 'tibia': self.left_legs_kit.servo[8]}         # Leg 5
        ]
        
        # Set pulse width range for all servos
        for leg in self.legs:
            leg['coxa'].set_pulse_width_range(self.servo_min_pulse, self.servo_max_pulse)
            leg['femur'].set_pulse_width_range(self.servo_min_pulse, self.servo_max_pulse)
            leg['tibia'].set_pulse_width_range(self.servo_min_pulse, self.servo_max_pulse)
        
        # Leg parameters
        self.coxa_length = 100
        self.femur_length = 100
        self.tibia_length = 100
        
        # Current angles for each leg (coxa, femur, tibia)
        self.current_angles = [[90, 60, 50] for _ in range(self.num_legs)]
        
        # Load offsets from file or create empty if not exists
        self.offsets_file = 'servo_offsets.json'
        self.servo_offsets = self._load_offsets()
        
        # Tripod gait groups
        self.tripod1 = [0, 2, 4]  # Right front, right rear, left middle
        self.tripod2 = [1, 3, 5]  # Right middle, left front, left rear
        
        self.tripod1_lifted = False
        self.tripod2_lifted = False
        
        # Movement speed
        self.movement_delay = 0.1
        
        # Default positions for walking
        self.default_femur_angle = 60
        self.default_tibia_angle = 50

    def simple_calibrate(self):
        """Simple calibration - set all servos to 90 degrees with offsets applied"""
        print("Starting simple calibration...")
        print("Setting all servos to 90 degrees with current offsets")
        
        for leg_num in range(self.num_legs):
            self._move_servo(leg_num, 'coxa', 90)
            self._move_servo(leg_num, 'femur', 90)
            self._move_servo(leg_num, 'tibia', 90)
        
        print("Simple calibration complete. All servos at 90 degrees (with offsets applied).")

    def _smooth_move_legs(self, leg_nums, joint, start_angles, end_angles, steps=30):
        """Move multiple legs' same joint simultaneously"""
        step_sizes = [(end_angles[i] - start_angles[i]) / steps for i in range(len(leg_nums))]
        
        for step in range(1, steps + 1):
            for i, leg_num in enumerate(leg_nums):
                angle = start_angles[i] + (step_sizes[i] * step)
                self._move_servo(leg_num, joint, angle)
            time.sleep(self.movement_delay / steps)

    def _load_offsets(self):
        """Load servo offsets from file or create new if not exists"""
        if os.path.exists(self.offsets_file):
            try:
                with open(self.offsets_file, 'r') as f:
                    offsets = json.load(f)
                    if len(offsets) == self.num_legs and all(isinstance(leg, dict) for leg in offsets):
                        return offsets
                    else:
                        print("Invalid offsets file structure. Creating new offsets.")
            except Exception as e:
                print(f"Error loading offsets: {e}. Creating new offsets.")
        
        new_offsets = []
        for _ in range(self.num_legs):
            new_offsets.append({'coxa': 0, 'femur': 0, 'tibia': 0})
        return new_offsets

    def _save_offsets(self):
        """Save servo offsets to file"""
        try:
            with open(self.offsets_file, 'w') as f:
                json.dump(self.servo_offsets, f, indent=4)
            print(f"Offsets saved to {self.offsets_file}")
        except Exception as e:
            print(f"Error saving offsets: {e}")

    def _apply_offset(self, leg_num, joint, angle):
        """Apply offset to servo angle with limits"""
        offset = self.servo_offsets[leg_num][joint]
        result = angle + offset
        return max(0, min(180, result))

    def _move_servo(self, leg_num, joint, angle):
        """Move servo to specified angle with offset applied"""
        angle = max(0, min(180, angle))  # Clamp angle to valid range
        servo = self.legs[leg_num][joint]
        servo.angle = self._apply_offset(leg_num, joint, angle)
        
        # Update current angle (without offset)
        idx = ['coxa', 'femur', 'tibia'].index(joint)
        self.current_angles[leg_num][idx] = angle

    def interactive_calibration(self):
        """Interactive calibration of all servos"""
        print("\n=== Servo Calibration Mode ===")
        print("For each servo, you'll be asked to enter an offset.")
        print("The servo will move to 90 degrees + offset.")
        print("Then you'll be asked if the servo is physically at 90 degrees.")
        print("If yes, the offset will be saved. If no, you can try again.")
        print("Press Ctrl+C at any time to exit calibration.\n")
        
        input("Press Enter to start calibration...")
        
        for leg_num in range(self.num_legs):
            for joint in ['coxa', 'femur', 'tibia']:
                self._calibrate_servo(leg_num, joint)
        
        self._save_offsets()
        print("\nCalibration complete! All offsets saved.")

    def _calibrate_servo(self, leg_num, joint):
        """Calibrate a single servo"""
        while True:
            try:
                print(f"\nCalibrating Leg {leg_num} {joint} servo")
                print(f"Current offset: {self.servo_offsets[leg_num][joint]}")
                
                # Move to neutral position without offset
                self._move_servo(leg_num, joint, 90)
                time.sleep(0.5)
                
                # Get user input for offset
                while True:
                    try:
                        offset_input = input("Enter new offset (-45 to +45, 's' to skip): ").strip()
                        if offset_input.lower() == 's':
                            print("Skipping this servo.")
                            return
                        
                        offset = int(offset_input)
                        if -45 <= offset <= 45:
                            break
                        print("Offset must be between -45 and +45!")
                    except ValueError:
                        print("Please enter a valid integer or 's' to skip.")
                
                # Apply temporary offset and move servo
                self.servo_offsets[leg_num][joint] = offset
                self._move_servo(leg_num, joint, 90)
                time.sleep(1)
                
                # Verify with user
                while True:
                    response = input("Does the servo look like it's at 90 degrees now? (y/n/s): ").lower()
                    if response in ['y', 'n', 's']:
                        break
                    print("Please enter 'y', 'n' or 's'")
                
                if response == 'y':
                    print(f"Offset {offset} saved for Leg {leg_num} {joint}")
                    break
                elif response == 'n':
                    print("Let's try again...")
                    continue
                else:
                    print("Skipping this servo.")
                    return
                    
            except KeyboardInterrupt:
                print("\nCalibration interrupted.")
                self._save_offsets()
                raise

    def _set_start_angles(self):
        """Set initial angles with offsets applied"""
        print("Setting initial positions with offsets...")
        for leg_num in range(self.num_legs):
            if leg_num in [0, 1, 2]:
                self._move_servo(leg_num, 'coxa', 90)
                self._move_servo(leg_num, 'femur', self.default_femur_angle)
                self._move_servo(leg_num, 'tibia', self.default_tibia_angle)
            else:
                self._move_servo(leg_num, 'coxa', 90)
                self._move_servo(leg_num, 'femur', 180 - self.default_femur_angle)
                self._move_servo(leg_num, 'tibia', 180 - self.default_tibia_angle)
        time.sleep(1)

    def lift_tripod(self, leg_nums, lift_height=20):
        """Lift all legs in tripod simultaneously"""
        start_angles = [self.current_angles[leg_num][1] for leg_num in leg_nums]
        
        # Calculate target femur angles
        target_angles = []
        for i, leg_num in enumerate(leg_nums):
            if leg_num in [0, 1, 2]:  # Right legs
                target_angles.append(start_angles[i] - lift_height)
            else:  # Left legs
                target_angles.append(start_angles[i] + lift_height)
        
        # Move all femurs simultaneously
        self._smooth_move_legs(leg_nums, 'femur', start_angles, target_angles)
        
        # Adjust tibias after femurs have moved
        start_tibia_angles = [self.current_angles[leg_num][2] for leg_num in leg_nums]
        target_tibia_angles = []
        for i, leg_num in enumerate(leg_nums):
            if leg_num in [0, 1, 2]:  # Right legs
                target_tibia_angles.append(start_tibia_angles[i] + (lift_height * 0.75))
            else:  # Left legs
                target_tibia_angles.append(start_tibia_angles[i] - (lift_height * 0.75))
        
        self._smooth_move_legs(leg_nums, 'tibia', start_tibia_angles, target_tibia_angles)

    def lower_tripod(self, leg_nums):
        """Lower all legs in tripod to default position"""
        # Get current angles
        start_femur_angles = [self.current_angles[leg_num][1] for leg_num in leg_nums]
        start_tibia_angles = [self.current_angles[leg_num][2] for leg_num in leg_nums]
        
        # Calculate target angles
        target_femur_angles = []
        target_tibia_angles = []
        for leg_num in leg_nums:
            if leg_num in [0, 1, 2]:
                target_femur_angles.append(self.default_femur_angle)
                target_tibia_angles.append(self.default_tibia_angle)
            else:
                target_femur_angles.append(180 - self.default_femur_angle)
                target_tibia_angles.append(180 - self.default_tibia_angle)
        
        # Move femurs and tibias simultaneously
        self._smooth_move_legs(leg_nums, 'femur', start_femur_angles, target_femur_angles)
        self._smooth_move_legs(leg_nums, 'tibia', start_tibia_angles, target_tibia_angles)

    def move_tripod_forward(self, leg_nums, step_length=30):
        """Move all legs in tripod forward simultaneously"""
        start_coxa_angles = [self.current_angles[leg_num][0] for leg_num in leg_nums]
        target_coxa_angles = []
        
        for leg_num in leg_nums:
            if leg_num in [0, 1, 2]:  # Right legs
                target_angle = start_coxa_angles[leg_nums.index(leg_num)] + step_length
            else:  # Left legs
                target_angle = start_coxa_angles[leg_nums.index(leg_num)] - step_length
            target_angle = max(30, min(150, target_angle))
            target_coxa_angles.append(target_angle)
        
        self._smooth_move_legs(leg_nums, 'coxa', start_coxa_angles, target_coxa_angles)

    def move_tripod_backward(self, leg_nums, step_length=30):
        """Move all legs in tripod backward simultaneously"""
        start_coxa_angles = [self.current_angles[leg_num][0] for leg_num in leg_nums]
        target_coxa_angles = []
        
        for leg_num in leg_nums:
            if leg_num in [0, 1, 2]:  # Right legs
                target_angle = start_coxa_angles[leg_nums.index(leg_num)] - step_length
            else:  # Left legs
                target_angle = start_coxa_angles[leg_nums.index(leg_num)] + step_length
            target_angle = max(30, min(150, target_angle))
            target_coxa_angles.append(target_angle)
        
        self._smooth_move_legs(leg_nums, 'coxa', start_coxa_angles, target_coxa_angles)

    def step_forward(self, step_length=45, step_height=20):
        """Improved walking gait with simultaneous leg movement"""
        # 1. Lift first tripod
        if self.tripod1_lifted == False:
            self.lift_tripod(self.tripod1, step_height)
            self.tripod1_lifted = True
            time.sleep(self.movement_delay)
        
        # 2. Move them forward
        self.move_tripod_forward(self.tripod1, step_length)
        time.sleep(self.movement_delay)
        
        # 3. Lower first tripod
        self.lower_tripod(self.tripod1)
        self.tripod1_lifted = False
        time.sleep(self.movement_delay)
        
        # 4. Lift second tripod
        self.lift_tripod(self.tripod2, step_height)
        self.tripod2_lifted = True
        time.sleep(self.movement_delay)
        
        # 5. Pull body with first tripod
        self.move_tripod_backward(self.tripod1, step_length)
        
        # 6. Move second tripod forward
        self.move_tripod_forward(self.tripod2, step_length)
        time.sleep(self.movement_delay)
        
        # 7. Lower second tripod
        self.lower_tripod(self.tripod2)
        self.tripod2_lifted = False
        time.sleep(self.movement_delay)
        
        # 8. Lift first tripod
        self.lift_tripod(self.tripod1, step_height)
        self.tripod1_lifted = True
        time.sleep(self.movement_delay)
        
        # 9. Pull body with second tripod
        self.move_tripod_backward(self.tripod2, step_length)
        
        # 10. Lower first tripod
        # self.lower_tripod(self.tripod1)
        # time.sleep(self.movement_delay)
    
    def turn_in_place(self, direction='right', turn_angle=15, step_height=20):
        """
        Rotate the spider in place by moving legs in opposite directions.
        
        Args:
            direction (str): 'right' or 'left' - direction of rotation
            turn_angle (int): angle in degrees for coxa movement
            step_height (int): height to lift legs during turn
        """
        try:
            # Determine rotation directions based on turn direction
            if direction.lower() == 'right':
                right_dir = -turn_angle  # Right legs move backward
                left_dir = turn_angle     # Left legs move forward
            else:
                right_dir = turn_angle    # Right legs move forward
                left_dir = -turn_angle    # Left legs move backward

            # 1. Lift first tripod
            self.lift_tripod(self.tripod1, step_height)
            time.sleep(self.movement_delay)
            
            # 2. Move lifted legs - right legs backward, left legs forward
            start_coxa_angles = [self.current_angles[leg_num][0] for leg_num in self.tripod1]
            target_coxa_angles = []
            for leg_num in self.tripod1:
                if leg_num in [0, 1, 2]:  # Right legs
                    target_angle = start_coxa_angles[self.tripod1.index(leg_num)] + right_dir
                else:  # Left legs
                    target_angle = start_coxa_angles[self.tripod1.index(leg_num)] - left_dir
                target_angle = max(30, min(150, target_angle))
                target_coxa_angles.append(target_angle)
            
            self._smooth_move_legs(self.tripod1, 'coxa', start_coxa_angles, target_coxa_angles)
            time.sleep(self.movement_delay)
            
            # 3. Lower first tripod
            self.lower_tripod(self.tripod1)
            time.sleep(self.movement_delay)
            
            # 4. Lift second tripod
            self.lift_tripod(self.tripod2, step_height)
            time.sleep(self.movement_delay)
            
             # 5. Move grounded legs to rotate 
            start_coxa_angles = [self.current_angles[leg_num][0] for leg_num in self.tripod1]
            target_coxa_angles = []
            
            for leg_num in self.tripod1:
                if leg_num in [0, 1, 2]:  # Right legs
                    target_angle = start_coxa_angles[self.tripod1.index(leg_num)] - right_dir
                else:  # Left legs
                    target_angle = start_coxa_angles[self.tripod1.index(leg_num)] + left_dir
                target_angle = max(30, min(150, target_angle))
                target_coxa_angles.append(target_angle)
            
            self._smooth_move_legs(self.tripod1, 'coxa', start_coxa_angles, target_coxa_angles)
            time.sleep(self.movement_delay)
            
            # 6. Move second tripod - right legs backward, left legs forward
            start_coxa_angles = [self.current_angles[leg_num][0] for leg_num in self.tripod2]
            target_coxa_angles = []
            
            for leg_num in self.tripod2:
                if leg_num in [0, 1, 2]:  # Right legs
                    target_angle = start_coxa_angles[self.tripod2.index(leg_num)] + right_dir
                else:  # Left legs
                    target_angle = start_coxa_angles[self.tripod2.index(leg_num)] - left_dir
                target_angle = max(30, min(150, target_angle))
                target_coxa_angles.append(target_angle)
            
            self._smooth_move_legs(self.tripod2, 'coxa', start_coxa_angles, target_coxa_angles)
            time.sleep(self.movement_delay)
            
            # 7. Lower second tripod
            self.lower_tripod(self.tripod2)
            time.sleep(self.movement_delay)
            
            # 8. Lift first tripod
            self.lift_tripod(self.tripod1)
            time.sleep(self.movement_delay)
            
            # 9. Rotate grounded legs (tripod2)
            start_coxa_angles = [self.current_angles[leg_num][0] for leg_num in self.tripod2]
            target_coxa_angles = []
            
            for leg_num in self.tripod2:
                if leg_num in [0, 1, 2]:  # Right legs
                    target_angle = start_coxa_angles[self.tripod2.index(leg_num)] - right_dir
                else:  # Left legs
                    target_angle = start_coxa_angles[self.tripod2.index(leg_num)] + left_dir
                target_angle = max(30, min(150, target_angle))
                target_coxa_angles.append(target_angle)
            
            self._smooth_move_legs(self.tripod2, 'coxa', start_coxa_angles, target_coxa_angles)
            time.sleep(self.movement_delay)
            
            # 10. Lower first tripod legs
            self.lower_tripod(self.tripod1)
            time.sleep(self.movement_delay)          
            
            
        except Exception as e:
            print(f"Error during turn: {e}")
            self._set_start_angles()

def main():
    parser = argparse.ArgumentParser(description='Spider Robot Control')
    parser.add_argument('--mode', choices=['calibrate', 'simple_calibrate', 'walk', 'turn'], required=True,
                       help='Operation mode: "calibrate" for interactive calibration, '
                            '"simple_calibrate" for quick 90-degree calibration, '
                            '"walk" for walking, '
                            '"turn" for turning in place')
    parser.add_argument('--direction', choices=['left', 'right'], 
                       help='Turn direction (only for turn mode)')
    args = parser.parse_args()
    
    spider = SpiderRobot()
    
    if args.mode == 'calibrate':
        spider.interactive_calibration()
    elif args.mode == 'simple_calibrate':
        spider.simple_calibrate()
    elif args.mode == 'walk':
        try:
            spider._set_start_angles()
            print("Spider is ready to walk! Press Ctrl+C to stop.")
            while True:
                spider.step_forward(step_length=15, step_height=25)
        except KeyboardInterrupt:
            print("\nReturning to initial position...")
            spider._set_start_angles()
            print("Done!")
    elif args.mode == 'turn':
        if not args.direction:
            print("Error: --direction argument is required for turn mode")
            return
        try:
            spider._set_start_angles()
            print(f"Spider is ready to turn {args.direction}! Press Ctrl+C to stop.")
            while True:
                spider.turn_in_place(direction=args.direction, turn_angle=15, step_height=25)
        except KeyboardInterrupt:
            print("\nReturning to initial position...")
            spider._set_start_angles()
            print("Done!")

if __name__ == "__main__":
    main()