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
        self.current_angles = [[90, 60, 90] for _ in range(self.num_legs)]  # Изменено начальное положение для "провисания"
        
        # Load offsets from file or create empty if not exists
        self.offsets_file = 'servo_offsets.json'
        self.servo_offsets = self._load_offsets()
        
        # Tripod gait groups
        self.tripod1 = [0, 2, 4]  # Right front, right rear, left middle
        self.tripod2 = [1, 3, 5]  # Right middle, left front, left rear
        
        # Movement speed
        self.movement_delay = 0.3  # Уменьшена задержка для более плавного движения
        
        # Default positions for walking
        self.default_femur_angle = 60  # Изменено для "провисания"
        self.default_tibia_angle = 90  # Изменено для "провисания"
        
    def _set_start_angles(self):
        """Set initial angles with offsets applied"""
        for leg_num in range(self.num_legs):
            if leg_num in [0, 1, 2]:
                self._move_servo(leg_num, 'coxa', 90)
                self._move_servo(leg_num, 'femur', self.default_femur_angle)
                self._move_servo(leg_num, 'tibia', self.default_tibia_angle)
            else:
                self._move_servo(leg_num, 'coxa', 90)
                self._move_servo(leg_num, 'femur', 180 - self.default_femur_angle)
                self._move_servo(leg_num, 'tibia', 180 - self.default_tibia_angle)
    
    def _load_offsets(self):
        """Load servo offsets from file"""
        if os.path.exists(self.offsets_file):
            with open(self.offsets_file, 'r') as f:
                return json.load(f)
        else:
            # Create empty offsets structure
            offsets = []
            for _ in range(self.num_legs):
                offsets.append({'coxa': 0, 'femur': 0, 'tibia': 0})
            return offsets
    
    def _save_offsets(self):
        """Save servo offsets to file"""
        with open(self.offsets_file, 'w') as f:
            json.dump(self.servo_offsets, f, indent=4)
    
    def _apply_offset(self, leg_num, joint, angle):
        """Apply offset to servo angle"""
        offset = self.servo_offsets[leg_num][joint]
        return angle + offset
    
    def _move_servo(self, leg_num, joint, angle):
        """Move servo to specified angle with offset applied"""
        angle = max(0, min(180, angle))  # Ограничение угла
        servo = self.legs[leg_num][joint]
        servo.angle = self._apply_offset(leg_num, joint, angle)
        
        # Update current angle (without offset)
        idx = ['coxa', 'femur', 'tibia'].index(joint)
        self.current_angles[leg_num][idx] = angle
    
    def calibrate(self):
        """Calibrate all servos to center position (90 degrees) with offsets"""
        print("Starting servo calibration...")
        
        for leg_num in range(self.num_legs):
            self._move_servo(leg_num, 'coxa', 90)
            self._move_servo(leg_num, 'femur', 90)
            self._move_servo(leg_num, 'tibia', 90)
        
        print("Calibration complete. All servos at 90 degrees (with offsets applied).")
    
    def calibrate_offsets(self):
        """Interactive calibration of servo offsets"""
        print("Starting interactive servo offset calibration...")
        print("For each servo, you'll be asked to enter an offset.")
        print("The servo will move to 90 degrees + offset.")
        print("Then you'll be asked if the servo is at 90 degrees physically.")
        print("If yes, the offset will be saved. If no, you can try again.")
        
        for leg_num in range(self.num_legs):
            for joint in ['coxa', 'femur', 'tibia']:
                self._calibrate_single_servo(leg_num, joint)
        
        self._save_offsets()
        print("Offset calibration complete. Offsets saved to", self.offsets_file)
    
    def _calibrate_single_servo(self, leg_num, joint):
        """Calibrate offset for a single servo"""
        while True:
            try:
                # Get current angle without offset
                current_angle = self.current_angles[leg_num][['coxa', 'femur', 'tibia'].index(joint)]
                
                # Ask user for offset
                offset = int(input(f"Enter offset for leg {leg_num} {joint} (current: {self.servo_offsets[leg_num][joint]}): ") or 0)
                
                # Apply offset temporarily
                self.servo_offsets[leg_num][joint] = offset
                
                # Move servo to 90 degrees with new offset
                target_angle = 90
                self._move_servo(leg_num, joint, target_angle)
                
                # Ask user if position is correct
                response = input("Does the servo look like it's at 90 degrees now? (y/n): ").lower()
                if response == 'y':
                    print(f"Offset {offset} saved for leg {leg_num} {joint}")
                    break
                else:
                    # Reset to previous angle
                    self._move_servo(leg_num, joint, current_angle)
                    print("Let's try again...")
            except ValueError:
                print("Please enter a valid integer for the offset.")
    
    def move_leg(self, leg_num, coxa_angle, femur_angle, tibia_angle):
        """Move leg to specified angles (offsets will be applied automatically)"""
        self._move_servo(leg_num, 'coxa', coxa_angle)
        self._move_servo(leg_num, 'femur', femur_angle)
        self._move_servo(leg_num, 'tibia', tibia_angle)
    
    def lift_leg(self, leg_num, lift_height=20):
        """Lift leg to specified height"""
        coxa_angle, femur_angle, tibia_angle = self.current_angles[leg_num]
        
        # Different lifting logic for right and left legs
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
        self.move_leg(leg_num, coxa_angle, self.default_femur_angle, self.default_tibia_angle)
    
    def move_leg_forward(self, leg_num, step_length=15):
        """Move leg forward"""
        coxa_angle, femur_angle, tibia_angle = self.current_angles[leg_num]
        
        # Different movement for right and left legs
        if leg_num in [0, 1, 2]:  # Right legs
            new_coxa = coxa_angle + step_length  # Исправлено направление движения
        else:  # Left legs
            new_coxa = coxa_angle - step_length  # Исправлено направление движения
        
        new_coxa = max(30, min(150, new_coxa))
        self.move_leg(leg_num, new_coxa, femur_angle, tibia_angle)
    
    def move_leg_backward(self, leg_num, step_length=15):
        """Move leg backward (used for pushing the body forward)"""
        coxa_angle, femur_angle, tibia_angle = self.current_angles[leg_num]
        
        if leg_num in [0, 1, 2]:  # Right legs
            new_coxa = coxa_angle - step_length  # Исправлено направление движения
        else:  # Left legs
            new_coxa = coxa_angle + step_length  # Исправлено направление движения
        
        new_coxa = max(30, min(150, new_coxa))
        self.move_leg(leg_num, new_coxa, femur_angle, tibia_angle)
    
    def push_body_forward(self, leg_group, step_length=15):
        """Push the body forward using legs that are on the ground"""
        for leg_num in leg_group:
            self.move_leg_backward(leg_num, step_length)
    
    def step_forward(self, step_length=15, step_height=20):
        """Улучшенная походка"""
        # 1. Поднять первую тройку ног
        for leg_num in self.tripod1:
            self.lift_leg(leg_num, step_height)
        time.sleep(self.movement_delay)
        
        # 2. Передвинуть их вперед
        for leg_num in self.tripod1:
            self.move_leg_forward(leg_num, step_length)
        time.sleep(self.movement_delay)
        
        # 3. Опустить первую тройку ног
        for leg_num in self.tripod1:
            self.lower_leg(leg_num)
        time.sleep(self.movement_delay)
        
        # 4. Поднять вторую тройку ног
        for leg_num in self.tripod2:
            self.lift_leg(leg_num, step_height)
        time.sleep(self.movement_delay)
        
        # 5. Первой тройкой ног подтянуть себя до coxa=90° (без задержки)
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
        
        # 6. Передвинуть вторую тройку ног вперед
        for leg_num in self.tripod2:
            self.move_leg_forward(leg_num, step_length)
        time.sleep(self.movement_delay)
        
        # 7. Опустить вторую тройку ног
        for leg_num in self.tripod2:
            self.lower_leg(leg_num)
        time.sleep(self.movement_delay)
        
        # 8. Поднять первую тройку ног
        for leg_num in self.tripod1:
            self.lift_leg(leg_num, step_height)
        time.sleep(self.movement_delay)
        
        # 9. Второй тройкой ног подтянуть себя до coxa=90° (без задержки)
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
        
        # 10. Опустить первую тройку ног
        for leg_num in self.tripod1:
            self.lower_leg(leg_num)
        time.sleep(self.movement_delay)
    
    def turn_left(self, turn_angle=15):
        """Turn left with improved movement"""
        # Lift first tripod
        for leg_num in self.tripod1:
            self.lift_leg(leg_num, 20)
        time.sleep(self.movement_delay)
        
        # Rotate first tripod
        for leg_num in self.tripod1:
            coxa_angle, femur_angle, tibia_angle = self.current_angles[leg_num]
            if leg_num in [0, 1, 2]:  # Right legs
                new_coxa = coxa_angle + turn_angle
            else:  # Left legs
                new_coxa = coxa_angle - turn_angle
            self.move_leg(leg_num, new_coxa, femur_angle, tibia_angle)
        time.sleep(self.movement_delay)
        
        # Lower first tripod
        for leg_num in self.tripod1:
            self.lower_leg(leg_num)
        time.sleep(self.movement_delay)
        
        # Push body with second tripod
        for leg_num in self.tripod2:
            coxa_angle, femur_angle, tibia_angle = self.current_angles[leg_num]
            if leg_num in [0, 1, 2]:  # Right legs
                new_coxa = coxa_angle - turn_angle
            else:  # Left legs
                new_coxa = coxa_angle + turn_angle
            self.move_leg(leg_num, new_coxa, femur_angle, tibia_angle)
        time.sleep(self.movement_delay)
        
        # Lift second tripod
        for leg_num in self.tripod2:
            self.lift_leg(leg_num, 20)
        time.sleep(self.movement_delay)
        
        # Rotate second tripod back to center
        for leg_num in self.tripod2:
            coxa_angle, femur_angle, tibia_angle = self.current_angles[leg_num]
            self.move_leg(leg_num, 90, femur_angle, tibia_angle)
        time.sleep(self.movement_delay)
        
        # Lower second tripod
        for leg_num in self.tripod2:
            self.lower_leg(leg_num)
        time.sleep(self.movement_delay)
    
    def turn_right(self, turn_angle=15):
        """Turn right with improved movement"""
        # Lift first tripod
        for leg_num in self.tripod1:
            self.lift_leg(leg_num, 20)
        time.sleep(self.movement_delay)
        
        # Rotate first tripod
        for leg_num in self.tripod1:
            coxa_angle, femur_angle, tibia_angle = self.current_angles[leg_num]
            if leg_num in [0, 1, 2]:  # Right legs
                new_coxa = coxa_angle - turn_angle
            else:  # Left legs
                new_coxa = coxa_angle + turn_angle
            self.move_leg(leg_num, new_coxa, femur_angle, tibia_angle)
        time.sleep(self.movement_delay)
        
        # Lower first tripod
        for leg_num in self.tripod1:
            self.lower_leg(leg_num)
        time.sleep(self.movement_delay)
        
        # Push body with second tripod
        for leg_num in self.tripod2:
            coxa_angle, femur_angle, tibia_angle = self.current_angles[leg_num]
            if leg_num in [0, 1, 2]:  # Right legs
                new_coxa = coxa_angle + turn_angle
            else:  # Left legs
                new_coxa = coxa_angle - turn_angle
            self.move_leg(leg_num, new_coxa, femur_angle, tibia_angle)
        time.sleep(self.movement_delay)
        
        # Lift second tripod
        for leg_num in self.tripod2:
            self.lift_leg(leg_num, 20)
        time.sleep(self.movement_delay)
        
        # Rotate second tripod back to center
        for leg_num in self.tripod2:
            coxa_angle, femur_angle, tibia_angle = self.current_angles[leg_num]
            self.move_leg(leg_num, 90, femur_angle, tibia_angle)
        time.sleep(self.movement_delay)
        
        # Lower second tripod
        for leg_num in self.tripod2:
            self.lower_leg(leg_num)
        time.sleep(self.movement_delay)

def main():
    parser = argparse.ArgumentParser(description='Spider Robot Control')
    parser.add_argument('--mode', choices=['calibrate', 'walk'], required=True,
                       help='Operation mode: calibrate or walk')
    args = parser.parse_args()
    
    spider = SpiderRobot()
    
    if args.mode == 'calibrate':
        spider.calibrate()
    elif args.mode == 'walk':
        try:
            spider._set_start_angles()
            time.sleep(1)  # Даем время на установку начального положения
            while True:
                print("Forward step")
                spider.step_forward(step_length=20, step_height=25)
                time.sleep(0.3)
                
                # Uncomment to test turning
                #print("Turn left")
                #spider.turn_left()
                #time.sleep(1)
                
                #print("Turn right")
                #spider.turn_right()
                #time.sleep(1)
        except KeyboardInterrupt:
            print("\nReturning to initial position...")
            spider.calibrate()

if __name__ == "__main__":
    main()