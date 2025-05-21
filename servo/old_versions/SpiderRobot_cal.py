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
        self.current_angles = [[90, 60, 90] for _ in range(self.num_legs)]
        
        # Load offsets from file or create empty if not exists
        self.offsets_file = 'servo_offsets.json'
        self.servo_offsets = self._load_offsets()
        
        # Tripod gait groups
        self.tripod1 = [0, 2, 4]  # Right front, right rear, left middle
        self.tripod2 = [1, 3, 5]  # Right middle, left front, left rear
        
        # Movement speed
        self.movement_delay = 0.2
        
        # Default positions for walking
        self.default_femur_angle = 75
        self.default_tibia_angle = 100

    def simple_calibrate(self):
        """Simple calibration - set all servos to 90 degrees with offsets applied"""
        print("Starting simple calibration...")
        print("Setting all servos to 90 degrees with current offsets")
        
        for leg_num in range(self.num_legs):
            self._move_servo(leg_num, 'coxa', 90)
            self._move_servo(leg_num, 'femur', 90)
            self._move_servo(leg_num, 'tibia', 90)
        
        print("Simple calibration complete. All servos at 90 degrees (with offsets applied).")

    def _smooth_move(self, leg_num, joint, start_angle, end_angle, steps=20, smooth=True):
        """
        Smooth servo movement from start to end angle with offset applied
        Set smooth=False for immediate movement
        """
        if not smooth:
            self._move_servo(leg_num, joint, end_angle)
            return
            
        step_size = (end_angle - start_angle) / steps
        servo = self.legs[leg_num][joint]
        
        for i in range(1, steps + 1):
            angle = start_angle + (step_size * i)
            self._move_servo(leg_num, joint, angle)
            time.sleep(self.movement_delay / steps)
            
    
    def _smooth_move_multi(self, leg_num, joints, start_angle, end_angle, steps=20, smooth=True):
        """
        Smooth servo movement from start to end angle with offset applied
        Set smooth=False for immediate movement
        """
        if not smooth:
            self._move_servo(leg_num, joint, end_angle)
            return
            
        step_size = (end_angle - start_angle) / steps
        servo = self.legs[leg_num][joint]
        
        for i in range(1, steps + 1):
            angle = start_angle + (step_size * i)
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

    def _set_start_angles(self, smooth=True):
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

    def move_leg(self, leg_num, coxa_angle, femur_angle, tibia_angle, smooth=True):
        """Move leg to specified angles with offsets"""
        
        self._smooth_move(leg_num, 'coxa', self.current_angles[leg_num][0], coxa_angle, smooth=smooth)
        self._smooth_move(leg_num, 'femur', self.current_angles[leg_num][1], femur_angle, smooth=smooth)
        self._smooth_move(leg_num, 'tibia', self.current_angles[leg_num][2], tibia_angle, smooth=smooth)

    #def move_leg(self, leg_num, coxa_angle, femur_angle, tibia_angle):
        # """Move leg to specified angles with offsets"""
        # self._move_servo(leg_num, 'coxa', coxa_angle)
        # self._move_servo(leg_num, 'femur', femur_angle)
        # self._move_servo(leg_num, 'tibia', tibia_angle)

    def lift_leg(self, leg_num, lift_height=20):
        """Lift leg to specified height"""
        coxa_angle, femur_angle, tibia_angle = self.current_angles[leg_num]
        
        if leg_num in [0, 1, 2]:  # Right legs
            new_femur = femur_angle - lift_height
            new_tibia = tibia_angle + (lift_height * 0.75)
        else:  # Left legs
            new_femur = femur_angle + lift_height
            new_tibia = tibia_angle - (lift_height * 0.75)
        
        self.move_leg(leg_num, coxa_angle, new_femur, new_tibia)

    def lower_leg(self, leg_num):
        """Lower leg to default position"""
        coxa_angle, _, _ = self.current_angles[leg_num]
        if leg_num in [0, 1, 2]:
            self.move_leg(leg_num, coxa_angle, self.default_femur_angle, self.default_tibia_angle)
        else:
            self.move_leg(leg_num, coxa_angle, 180 - self.default_femur_angle, 180 - self.default_tibia_angle)

    def move_leg_forward(self, leg_num, step_length=15):
        """Move leg forward"""
        coxa_angle, femur_angle, tibia_angle = self.current_angles[leg_num]
        
        if leg_num in [0, 1, 2]:  # Right legs
            new_coxa = coxa_angle + step_length
        else:  # Left legs
            new_coxa = coxa_angle - step_length
        
        new_coxa = max(30, min(150, new_coxa))
        self.move_leg(leg_num, new_coxa, femur_angle, tibia_angle)

    def move_leg_backward(self, leg_num, step_length=15):
        """Move leg backward (used for pushing the body forward)"""
        coxa_angle, femur_angle, tibia_angle = self.current_angles[leg_num]
        
        if leg_num in [0, 1, 2]:  # Right legs
            new_coxa = coxa_angle - step_length
        else:  # Left legs
            new_coxa = coxa_angle + step_length
        
        new_coxa = max(30, min(150, new_coxa))
        self.move_leg(leg_num, new_coxa, femur_angle, tibia_angle)

    def step_forward(self, step_length=15, step_height=20):
        """Improved walking gait"""
        # 1. Lift first tripod
        for leg_num in self.tripod1:
            self.lift_leg(leg_num, step_height)
        time.sleep(self.movement_delay)
        
        # 2. Move them forward
        for leg_num in self.tripod1:
            self.move_leg_forward(leg_num, step_length)
        time.sleep(self.movement_delay)
        
        # 3. Lower first tripod
        for leg_num in self.tripod1:
            self.lower_leg(leg_num)
        time.sleep(self.movement_delay)
        
        # 4. Lift second tripod
        for leg_num in self.tripod2:
            self.lift_leg(leg_num, step_height)
        time.sleep(self.movement_delay)
        
        # 5. Pull body with first tripod (no delay)
        for leg_num in self.tripod1:
            current_coxa = self.current_angles[leg_num][0]
            if leg_num in [0, 1, 2]:  # Right legs
                if current_coxa > 90:
                    self.move_leg_backward(leg_num, current_coxa - 90)
                elif current_coxa < 90:
                    self.move_leg_forward(leg_num, 90 - current_coxa)
            else:  # Left legs
                if current_coxa < 90:
                    self.move_leg_backward(leg_num, 90 - current_coxa)
                elif current_coxa > 90:
                    self.move_leg_forward(leg_num, current_coxa - 90)
        
        # 6. Move second tripod forward
        for leg_num in self.tripod2:
            self.move_leg_forward(leg_num, step_length)
        time.sleep(self.movement_delay)
        
        # 7. Lower second tripod
        for leg_num in self.tripod2:
            self.lower_leg(leg_num)
        time.sleep(self.movement_delay)
        
        # 8. Lift first tripod
        for leg_num in self.tripod1:
            self.lift_leg(leg_num, step_height)
        time.sleep(self.movement_delay)
        
        # 9. Pull body with second tripod (no delay)
        for leg_num in self.tripod2:
            current_coxa = self.current_angles[leg_num][0]
            if leg_num in [0, 1, 2]:  # Right legs
                if current_coxa > 90:
                    self.move_leg_backward(leg_num, current_coxa - 90)
                elif current_coxa < 90:
                    self.move_leg_forward(leg_num, 90 - current_coxa)
            else:  # Left legs
                if current_coxa < 90:
                    self.move_leg_backward(leg_num, 90 - current_coxa)
                elif current_coxa > 90:
                    self.move_leg_forward(leg_num, current_coxa - 90)
        
        # 10. Lower first tripod
        for leg_num in self.tripod1:
            self.lower_leg(leg_num)
        time.sleep(self.movement_delay)

def main():
    parser = argparse.ArgumentParser(description='Spider Robot Control')
    parser.add_argument('--mode', choices=['calibrate', 'simple_calibrate', 'walk'], required=True,
                       help='Operation mode: "calibrate" for interactive calibration, '
                            '"simple_calibrate" for quick 90-degree calibration, '
                            'or "walk" for walking')
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
                spider.step_forward(step_length=20, step_height=25)
                time.sleep(0.3)
        except KeyboardInterrupt:
            print("\nReturning to initial position...")
            spider._set_start_angles()
            print("Done!")

if __name__ == "__main__":
    main()