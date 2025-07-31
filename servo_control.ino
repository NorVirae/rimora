#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create servo driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo configuration
#define SERVOMIN  150 // Minimum pulse length count (out of 4096)
#define SERVOMAX  600 // Maximum pulse length count (out of 4096)
#define USMIN  600    // Minimum microsecond length based on MG90S specs
#define USMAX  2400   // Maximum microsecond length based on MG90S specs

// Servo pin assignments on PCA9685
#define BASE_SERVO     0  // Base rotation
#define SHOULDER_SERVO 1  // Shoulder joint
#define ELBOW_SERVO    2  // Elbow joint
#define WRIST_SERVO    3  // Wrist rotation
#define GRIPPER_SERVO  4  // Gripper open/close

// Current servo positions (0-180 degrees)
int servoPositions[5] = {90, 90, 90, 90, 90};

// Target positions for smooth movement
int targetPositions[5] = {90, 90, 90, 90, 90};

// Movement speed (degrees per step)
int moveSpeed = 2;

void setup() {
  Serial.begin(9600);
  Serial.println("Robotic Arm Control System Starting...");
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50); // MG90S servos work at 50Hz
  
  delay(10);
  
  // Initialize all servos to home position
  moveToHome();
  
  Serial.println("System Ready!");
  Serial.println("Commands:");
  Serial.println("H - Home position");
  Serial.println("P - Pick position");
  Serial.println("D - Drop position");
  Serial.println("G1 - Open gripper");
  Serial.println("G0 - Close gripper");
  Serial.println("M<servo><angle> - Move servo (e.g., M0090 = base to 90°)");
}

void loop() {
  // Smooth movement update
  updateServoMovement();
  
  // Handle serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
  }
  
  delay(20); // Small delay for smooth movement
}

void processCommand(String cmd) {
  cmd.toUpperCase();
  
  if (cmd == "H") {
    moveToHome();
    Serial.println("Moving to home position");
  }
  else if (cmd == "P") {
    moveToPickPosition();
    Serial.println("Moving to pick position");
  }
  else if (cmd == "D") {
    moveToDropPosition();
    Serial.println("Moving to drop position");
  }
  else if (cmd == "G1") {
    openGripper();
    Serial.println("Opening gripper");
  }
  else if (cmd == "G0") {
    closeGripper();
    Serial.println("Closing gripper");
  }
  else if (cmd.startsWith("M") && cmd.length() == 5) {
    // Format: M<servo><angle> e.g., M0090
    int servoNum = cmd.substring(1, 2).toInt();
    int angle = cmd.substring(2, 5).toInt();
    
    if (servoNum >= 0 && servoNum <= 4 && angle >= 0 && angle <= 180) {
      moveServo(servoNum, angle);
      Serial.println("Moving servo " + String(servoNum) + " to " + String(angle) + "°");
    }
  }
  else {
    Serial.println("Unknown command: " + cmd);
  }
}

void moveServo(int servo, int angle) {
  if (servo >= 0 && servo <= 4 && angle >= 0 && angle <= 180) {
    targetPositions[servo] = angle;
  }
}

void setServoAngle(int servo, int angle) {
  int pulseWidth = map(angle, 0, 180, USMIN, USMAX);
  int pwmValue = int(float(pulseWidth) / 1000000 * 50 * 4096);
  pwm.setPWM(servo, 0, pwmValue);
  servoPositions[servo] = angle;
}

void updateServoMovement() {
  bool moving = false;
  
  for (int i = 0; i < 5; i++) {
    if (servoPositions[i] != targetPositions[i]) {
      moving = true;
      int diff = targetPositions[i] - servoPositions[i];
      
      if (abs(diff) <= moveSpeed) {
        servoPositions[i] = targetPositions[i];
      } else {
        servoPositions[i] += (diff > 0) ? moveSpeed : -moveSpeed;
      }
      
      setServoAngle(i, servoPositions[i]);
    }
  }
}

void moveToHome() {
  targetPositions[BASE_SERVO] = 90;      // Base center
  targetPositions[SHOULDER_SERVO] = 90;  // Shoulder up
  targetPositions[ELBOW_SERVO] = 90;     // Elbow neutral
  targetPositions[WRIST_SERVO] = 90;     // Wrist neutral
  targetPositions[GRIPPER_SERVO] = 90;   // Gripper half open
}

void moveToPickPosition() {
  targetPositions[BASE_SERVO] = 90;      // Base center
  targetPositions[SHOULDER_SERVO] = 45;  // Shoulder down
  targetPositions[ELBOW_SERVO] = 135;    // Elbow bent
  targetPositions[WRIST_SERVO] = 90;     // Wrist neutral
  targetPositions[GRIPPER_SERVO] = 180;  // Gripper open
}

void moveToDropPosition() {
  targetPositions[BASE_SERVO] = 45;      // Base rotated
  targetPositions[SHOULDER_SERVO] = 60;  // Shoulder mid
  targetPositions[ELBOW_SERVO] = 120;    // Elbow bent
  targetPositions[WRIST_SERVO] = 90;     // Wrist neutral
  targetPositions[GRIPPER_SERVO] = 0;    // Gripper closed
}

void openGripper() {
  targetPositions[GRIPPER_SERVO] = 180;
}

void closeGripper() {
  targetPositions[GRIPPER_SERVO] = 0;
}

// Function to get current servo positions (for external communication)
void printServoPositions() {
  Serial.print("POSITIONS:");
  for (int i = 0; i < 5; i++) {
    Serial.print(servoPositions[i]);
    if (i < 4) Serial.print(",");
  }
  Serial.println();
}