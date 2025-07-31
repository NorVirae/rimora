# Robotic Arm Hardware Setup and Integration Guide

## Hardware Components

### Required Components
- **Arduino Uno/Nano** - Main servo controller
- **Raspberry Pi 4** - High-level control and vision processing
- **Adafruit PCA9685 16-Channel PWM Driver** - Servo control board
- **5x MG90S Servo Motors** - Arm joints and gripper
- **USB Camera** - Vision system
- **Power Supply** - 5V/6V for servos (minimum 3A)
- **Jumper Wires** - Connections
- **Breadboard or PCB** - Circuit assembly

### Optional Components
- **External Power Supply** for servos (recommended for stable operation)
- **Level Shifter** (if using 3.3V Raspberry Pi with 5V Arduino)
- **Heat Sinks** for servos under heavy load

## Wiring Diagram

### Arduino to PCA9685 Connection
```
Arduino Uno    →    PCA9685
GND           →    GND
5V            →    VCC
A4 (SDA)      →    SDA
A5 (SCL)      →    SCL
```

### PCA9685 to Servos
```
Servo Function    →    PCA9685 Channel
Base Rotation     →    Channel 0
Shoulder Joint    →    Channel 1
Elbow Joint       →    Channel 2
Wrist Rotation    →    Channel 3
Gripper           →    Channel 4
```

### Servo Power Supply
- Connect servo power (5V/6V) to PCA9685 V+ terminal
- Ensure adequate current supply (each MG90S draws ~200mA under load)
- Use separate power supply for servos if drawing >500mA total

### Raspberry Pi to Arduino
```
Raspberry Pi    →    Arduino
GND            →    GND
GPIO 14 (TXD)  →    Pin 0 (RX)
GPIO 15 (RXD)  →    Pin 1 (TX)
```
*Or use USB connection for serial communication*

## Software Installation

### Arduino IDE Setup
1. Install Arduino IDE
2. Install Adafruit PWM Servo Driver Library:
   ```
   Sketch → Include Library → Manage Libraries
   Search: "Adafruit PWM Servo Driver"
   Install the library by Adafruit
   ```
3. Upload the Arduino servo control code

### Raspberry Pi Setup
1. **Enable Serial Communication:**
   ```bash
   sudo raspi-config
   # Navigate to Interface Options → Serial Port
   # Enable serial port hardware: Yes
   # Enable serial console: No
   ```

2. **Install Python Dependencies:**
   ```bash
   sudo apt update
   sudo apt install python3-pip python3-opencv
   pip3 install pyserial numpy opencv-python
   ```

3. **Set up project directory:**
   ```bash
   mkdir ~/robotic_arm
   cd ~/robotic_arm
   # Copy the Python files here
   ```

## Mechanical Assembly

### Servo Mounting Guidelines
1. **Base Servo (Channel 0):** Mount horizontally for base rotation
2. **Shoulder Servo (Channel 1):** Mount vertically for up/down movement
3. **Elbow Servo (Channel 2):** Connect to shoulder arm for bending
4. **Wrist Servo (Channel 3):** Mount for end-effector rotation
5. **Gripper Servo (Channel 4):** Control gripper open/close mechanism

### Mechanical Considerations
- Use servo horns and brackets for secure mounting
- Ensure adequate clearance between moving parts
- Balance the arm to reduce servo load
- Consider gear reduction for heavier loads

## Calibration Process

### 1. Servo Calibration
```cpp
// Test individual servo movement
// Upload Arduino code and use serial monitor
// Send commands: M0090, M1090, etc.
// Verify each servo moves to correct position
```

### 2. Camera Calibration
```python
# Run the vision system
python3 opencv_vision_system.py
# Choose option 3 for camera calibration
# Use printed chessboard pattern
```

### 3. Working Area Setup
- Define physical working area boundaries
- Calibrate pixel-to-world coordinate conversion
- Test pick and place accuracy

## Usage Instructions

### Basic Operation
1. **Start Arduino:** Upload and run servo control code
2. **Start Raspberry Pi Controller:**
   ```bash
   python3 raspberry_pi_controller.py
   ```
3. **Start Vision System:**
   ```bash
   python3 opencv_vision_system.py
   ```

### Manual Control Commands
```
H    - Home position
P    - Pick position  
D    - Drop position
G1   - Open gripper
G0   - Close gripper
M0090 - Move servo 0 to 90 degrees
```

### Autonomous Operation
1. Place colored objects in camera view
2. Start autonomous mode in vision system
3. System will automatically detect and pick objects

## Troubleshooting

### Common Issues

**Servos not moving:**
- Check power supply voltage and current
- Verify PCA9685 connections
- Test with basic servo sweep code

**Erratic servo movement:**
- Insufficient power supply
- Loose connections
- Interference from nearby electronics

**Camera not detected:**
- Check USB connection
- Verify camera device ID (usually 0 or 1)
- Test with: `ls /dev/video*`

**Serial communication issues:**
- Check baud rate matches (9600)
- Verify RX/TX connections
- Test with simple echo program

**Poor object detection:**
- Adjust lighting conditions
- Tune color detection ranges
- Calibrate camera properly

### Debug Commands
```bash
# Test camera
python3 -c "import cv2; print(cv2.VideoCapture(0).read())"

# Test serial connection
python3 -c "import serial; s=serial.Serial('/dev/ttyUSB0', 9600); print('Connected')"

# Monitor Arduino output
screen /dev/ttyUSB0 9600
```

## Performance Optimization

### Servo Performance
- Use external power supply for consistent performance
- Add capacitors to reduce electrical noise
- Implement smooth acceleration/deceleration curves

### Vision Performance
- Optimize camera resolution vs. processing speed
- Use multi-threading for real-time processing
- Implement predictive tracking algorithms

### System Integration
- Use separate threads for vision and control
- Implement error handling and recovery
- Add safety limits and emergency stops

## Safety Considerations

### Electrical Safety
- Use appropriate fuses and circuit protection
- Ensure proper grounding
- Avoid short circuits in power connections

### Mechanical Safety  
- Implement software position limits
- Add physical end stops if needed
- Emergency stop functionality
- Gradual speed ramping to prevent damage

### Operational Safety
- Clear working area of obstacles
- Supervised operation during testing
- Proper mounting and stability

## Expansion Ideas

### Hardware Upgrades
- Add force sensors for grip feedback
- Implement encoders for position feedback
- Upgrade to higher torque servos
- Add more degrees of freedom

### Software Enhancements
- Machine learning for better object recognition
- Voice control integration
- Web interface for remote operation
- Path planning algorithms
- Multiple object sorting capabilities

## File Structure
```
robotic_arm/
├── arduino_servo_control.ino      # Arduino code
├── raspberry_pi_controller.py     # Raspberry Pi interface
├── opencv_vision_system.py        # Vision processing
├── camera_calibration.json        # Camera calibration data
├── config/
│   ├── servo_limits.json         # Servo position limits
│   └── vision_config.json        # Vision system settings
└── docs/
    ├── hardware_setup.md          # This guide
    └── api_reference.md           # Code documentation
```

This setup provides a complete foundation for a vision-guided robotic arm capable of autonomous object manipulation. Start with basic manual control, then gradually integrate the vision system for autonomous operation.