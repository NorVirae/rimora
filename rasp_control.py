#!/usr/bin/env python3
"""
Raspberry Pi Robotic Arm Controller
Interfaces between Arduino servo controller and external systems
"""

import serial
import time
import json
import threading
from typing import List, Tuple, Dict, Optional

class RoboticArmController:
    def __init__(self, arduino_port='/dev/ttyUSB0', baudrate=9600):
        """
        Initialize the robotic arm controller
        
        Args:
            arduino_port: Serial port where Arduino is connected
            baudrate: Serial communication speed
        """
        self.arduino_port = arduino_port
        self.baudrate = baudrate
        self.serial_connection = None
        self.is_connected = False
        self.current_positions = [90, 90, 90, 90, 90]  # Base, Shoulder, Elbow, Wrist, Gripper
        self.is_moving = False
        
        # Servo limits (adjust based on your arm's physical constraints)
        self.servo_limits = {
            0: (0, 180),    # Base rotation
            1: (30, 150),   # Shoulder (prevent collision)
            2: (45, 180),   # Elbow
            3: (0, 180),    # Wrist
            4: (0, 180)     # Gripper
        }
        
        self.connect()
    
    def connect(self):
        """Establish serial connection with Arduino"""
        try:
            self.serial_connection = serial.Serial(
                self.arduino_port, 
                self.baudrate, 
                timeout=1
            )
            time.sleep(2)  # Wait for Arduino to initialize
            self.is_connected = True
            print(f"Connected to Arduino on {self.arduino_port}")
            
            # Start monitoring thread
            self.monitor_thread = threading.Thread(target=self._monitor_arduino, daemon=True)
            self.monitor_thread.start()
            
        except serial.SerialException as e:
            print(f"Failed to connect to Arduino: {e}")
            self.is_connected = False
    
    def disconnect(self):
        """Close serial connection"""
        if self.serial_connection and self.is_connected:
            self.serial_connection.close()
            self.is_connected = False
            print("Disconnected from Arduino")
    
    def _send_command(self, command: str) -> bool:
        """Send command to Arduino"""
        if not self.is_connected:
            print("Not connected to Arduino")
            return False
        
        try:
            self.serial_connection.write(f"{command}\n".encode())
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False
    
    def _monitor_arduino(self):
        """Monitor Arduino responses in separate thread"""
        while self.is_connected:
            try:
                if self.serial_connection.in_waiting > 0:
                    response = self.serial_connection.readline().decode().strip()
                    if response:
                        print(f"Arduino: {response}")
                        # Parse position updates if needed
                        if response.startswith("POSITIONS:"):
                            positions_str = response.split(":")[1]
                            self.current_positions = [int(x) for x in positions_str.split(",")]
                time.sleep(0.1)
            except Exception as e:
                print(f"Monitor error: {e}")
                break
    
    def move_to_home(self):
        """Move arm to home position"""
        return self._send_command("H")
    
    def move_to_pick_position(self):
        """Move arm to pick position"""
        return self._send_command("P")
    
    def move_to_drop_position(self):
        """Move arm to drop position"""
        return self._send_command("D")
    
    def open_gripper(self):
        """Open the gripper"""
        return self._send_command("G1")
    
    def close_gripper(self):
        """Close the gripper"""
        return self._send_command("G0")
    
    def move_servo(self, servo_id: int, angle: int) -> bool:
        """
        Move specific servo to angle
        
        Args:
            servo_id: Servo number (0-4)
            angle: Target angle (0-180)
        """
        if servo_id not in range(5):
            print(f"Invalid servo ID: {servo_id}")
            return False
        
        # Check limits
        min_angle, max_angle = self.servo_limits[servo_id]
        if not (min_angle <= angle <= max_angle):
            print(f"Angle {angle} out of range for servo {servo_id} ({min_angle}-{max_angle})")
            return False
        
        command = f"M{servo_id}{angle:03d}"
        return self._send_command(command)
    
    def move_multiple_servos(self, servo_angles: Dict[int, int], delay_between=0.1):
        """
        Move multiple servos with optional delay
        
        Args:
            servo_angles: Dictionary of {servo_id: angle}
            delay_between: Delay between servo commands
        """
        for servo_id, angle in servo_angles.items():
            self.move_servo(servo_id, angle)
            if delay_between > 0:
                time.sleep(delay_between)
    
    def pick_and_place_sequence(self, pick_angles: Optional[Dict[int, int]] = None,
                              place_angles: Optional[Dict[int, int]] = None):
        """
        Execute a complete pick and place sequence
        
        Args:
            pick_angles: Custom pick position angles
            place_angles: Custom place position angles
        """
        print("Starting pick and place sequence...")
        
        # Move to home
        self.move_to_home()
        time.sleep(2)
        
        # Open gripper
        self.open_gripper()
        time.sleep(1)
        
        # Move to pick position
        if pick_angles:
            self.move_multiple_servos(pick_angles)
        else:
            self.move_to_pick_position()
        time.sleep(3)
        
        # Close gripper (pick object)
        self.close_gripper()
        time.sleep(2)
        
        # Lift slightly
        self.move_servo(1, 60)  # Shoulder up
        time.sleep(1)
        
        # Move to place position
        if place_angles:
            self.move_multiple_servos(place_angles)
        else:
            self.move_to_drop_position()
        time.sleep(3)
        
        # Open gripper (release object)
        self.open_gripper()
        time.sleep(1)
        
        # Return to home
        self.move_to_home()
        time.sleep(2)
        
        print("Pick and place sequence completed!")
    
    def move_to_coordinates(self, x: float, y: float, z: float):
        """
        Move arm to Cartesian coordinates (basic inverse kinematics)
        Note: This is a simplified implementation - you may need more complex IK
        
        Args:
            x, y, z: Target coordinates in mm
        """
        # Basic inverse kinematics calculation
        # This is simplified - implement proper IK based on your arm dimensions
        
        # Calculate base rotation
        base_angle = int(math.degrees(math.atan2(y, x)))
        base_angle = max(0, min(180, base_angle + 90))  # Convert to servo range
        
        # Calculate reach distance
        reach = math.sqrt(x*x + y*y)
        
        # Simple 2-DOF IK for shoulder and elbow (assuming known link lengths)
        L1 = 100  # Shoulder to elbow length (adjust for your arm)
        L2 = 100  # Elbow to wrist length (adjust for your arm)
        
        # Calculate shoulder and elbow angles (simplified)
        target_height = z
        target_reach = reach
        
        # This is a basic calculation - implement proper IK for your specific arm geometry
        shoulder_angle = 90  # Placeholder
        elbow_angle = 90     # Placeholder
        
        # Move servos
        angles = {
            0: base_angle,
            1: shoulder_angle,
            2: elbow_angle,
            3: 90,  # Wrist neutral
        }
        
        self.move_multiple_servos(angles)
    
    def get_current_positions(self) -> List[int]:
        """Get current servo positions"""
        return self.current_positions.copy()
    
    def emergency_stop(self):
        """Emergency stop - move to safe position"""
        print("EMERGENCY STOP!")
        self.move_to_home()

# Example usage and testing functions
def main():
    """Main function for testing the robotic arm"""
    print("Initializing Robotic Arm Controller...")
    
    # Initialize arm controller
    arm = RoboticArmController()
    
    if not arm.is_connected:
        print("Failed to connect to Arduino. Check connection and port.")
        return
    
    try:
        while True:
            print("\n=== Robotic Arm Control ===")
            print("1. Home position")
            print("2. Pick position")
            print("3. Drop position")
            print("4. Open gripper")
            print("5. Close gripper")
            print("6. Manual servo control")
            print("7. Pick and place sequence")
            print("8. Exit")
            
            choice = input("Enter choice (1-8): ").strip()
            
            if choice == '1':
                arm.move_to_home()
            elif choice == '2':
                arm.move_to_pick_position()
            elif choice == '3':
                arm.move_to_drop_position()
            elif choice == '4':
                arm.open_gripper()
            elif choice == '5':
                arm.close_gripper()
            elif choice == '6':
                servo_id = int(input("Enter servo ID (0-4): "))
                angle = int(input("Enter angle (0-180): "))
                arm.move_servo(servo_id, angle)
            elif choice == '7':
                arm.pick_and_place_sequence()
            elif choice == '8':
                break
            else:
                print("Invalid choice!")
            
            time.sleep(0.5)
    
    except KeyboardInterrupt:
        print("\nShutting down...")
    
    finally:
        arm.disconnect()

if __name__ == "__main__":
    import math
    main()