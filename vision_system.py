#!/usr/bin/env python3
"""
OpenCV Vision System for Robotic Arm
Object detection and tracking for autonomous pick and place operations
"""

import cv2
import numpy as np
import json
import time
import threading
from typing import List, Tuple, Dict, Optional
import math

class VisionSystem:
    def __init__(self, camera_id=0, calibration_file='camera_calibration.json'):
        """
        Initialize the vision system
        
        Args:
            camera_id: Camera device ID
            calibration_file: Path to camera calibration data
        """
        self.camera_id = camera_id
        self.cap = None
        self.is_running = False
        self.current_frame = None
        self.detected_objects = []
        
        # Object detection parameters
        self.object_colors = {
            'red': ([0, 120, 70], [10, 255, 255]),
            'green': ([36, 25, 25], [86, 255, 255]),
            'blue': ([94, 80, 2], [120, 255, 255]),
            'yellow': ([20, 100, 100], [30, 255, 255])
        }
        
        # Camera calibration parameters (default values - calibrate your camera)
        self.camera_matrix = np.array([[800, 0, 320],
                                      [0, 800, 240],
                                      [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((4, 1))
        
        # Working area definition (in pixels)
        self.work_area = {
            'x_min': 100, 'x_max': 540,
            'y_min': 100, 'y_max': 380
        }
        
        # Load calibration if file exists
        self.load_calibration(calibration_file)
        
        # Initialize camera
        self.initialize_camera()
    
    def initialize_camera(self):
        """Initialize camera capture"""
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            if self.cap.isOpened():
                print(f"Camera {self.camera_id} initialized successfully")
                return True
            else:
                print(f"Failed to open camera {self.camera_id}")
                return False
        except Exception as e:
            print(f"Camera initialization error: {e}")
            return False
    
    def load_calibration(self, calibration_file):
        """Load camera calibration data"""
        try:
            with open(calibration_file, 'r') as f:
                cal_data = json.load(f)
                self.camera_matrix = np.array(cal_data['camera_matrix'])
                self.dist_coeffs = np.array(cal_data['dist_coeffs'])
                print("Camera calibration loaded")
        except FileNotFoundError:
            print("Calibration file not found, using default parameters")
        except Exception as e:
            print(f"Error loading calibration: {e}")
    
    def start_capture(self):
        """Start video capture in separate thread"""
        if not self.cap or not self.cap.isOpened():
            self.initialize_camera()
        
        self.is_running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        print("Vision system started")
    
    def stop_capture(self):
        """Stop video capture"""
        self.is_running = False
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        print("Vision system stopped")
    
    def _capture_loop(self):
        """Main capture loop running in separate thread"""
        while self.is_running and self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                self.current_frame = frame.copy()
                self.process_frame(frame)
            time.sleep(0.033)  # ~30 FPS
    
    def process_frame(self, frame):
        """Process frame for object detection"""
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Clear previous detections
        self.detected_objects = []
        
        # Detect objects by color
        for color_name, (lower, upper) in self.object_colors.items():
            objects = self.detect_color_objects(hsv, lower, upper, color_name)
            self.detected_objects.extend(objects)
        
        # Draw detection results
        self.draw_detections(frame)
        
        # Show frame
        cv2.imshow('Robotic Arm Vision', frame)
        cv2.waitKey(1)
    
    def detect_color_objects(self, hsv_frame, lower_bound, upper_bound, color_name):
        """Detect objects of specific color"""
        # Create mask
        lower = np.array(lower_bound)
        upper = np.array(upper_bound)
        mask = cv2.inRange(hsv_frame, lower, upper)
        
        # Remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        objects = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Minimum object size
                # Calculate centroid
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # Check if object is in working area
                    if self.is_in_work_area(cx, cy):
                        # Calculate bounding box
                        x, y, w, h = cv2.boundingRect(contour)
                        
                        objects.append({
                            'color': color_name,
                            'center': (cx, cy),
                            'area': area,
                            'bounding_box': (x, y, w, h),
                            'contour': contour
                        })
        
        return objects
    
    def is_in_work_area(self, x, y):
        """Check if point is within working area"""
        return (self.work_area['x_min'] <= x <= self.work_area['x_max'] and
                self.work_area['y_min'] <= y <= self.work_area['y_max'])
    
    def draw_detections(self, frame):
        """Draw detection results on frame"""
        # Draw working area
        cv2.rectangle(frame, 
                     (self.work_area['x_min'], self.work_area['y_min']),
                     (self.work_area['x_max'], self.work_area['y_max']),
                     (255, 255, 255), 2)
        
        # Draw detected objects
        for obj in self.detected_objects:
            center = obj['center']
            color = obj['color']
            bbox = obj['bounding_box']
            
            # Draw bounding box
            cv2.rectangle(frame, (bbox[0], bbox[1]), 
                         (bbox[0] + bbox[2], bbox[1] + bbox[3]), 
                         (0, 255, 0), 2)
            
            # Draw center point
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            
            # Draw label
            label = f"{color} ({center[0]}, {center[1]})"
            cv2.putText(frame, label, (center[0] - 50, center[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Draw info
        info_text = f"Objects detected: {len(self.detected_objects)}"
        cv2.putText(frame, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    def get_largest_object(self, color_filter=None):
        """Get the largest detected object, optionally filtered by color"""
        if not self.detected_objects:
            return None
        
        filtered_objects = self.detected_objects
        if color_filter:
            filtered_objects = [obj for obj in self.detected_objects 
                              if obj['color'] == color_filter]
        
        if not filtered_objects:
            return None
        
        return max(filtered_objects, key=lambda x: x['area'])
    
    def get_closest_object_to_center(self, color_filter=None):
        """Get object closest to center of working area"""
        if not self.detected_objects:
            return None
        
        filtered_objects = self.detected_objects
        if color_filter:
            filtered_objects = [obj for obj in self.detected_objects 
                              if obj['color'] == color_filter]
        
        if not filtered_objects:
            return None
        
        # Calculate center of working area
        center_x = (self.work_area['x_min'] + self.work_area['x_max']) // 2
        center_y = (self.work_area['y_min'] + self.work_area['y_max']) // 2
        
        def distance_to_center(obj):
            dx = obj['center'][0] - center_x
            dy = obj['center'][1] - center_y
            return math.sqrt(dx*dx + dy*dy)
        
        return min(filtered_objects, key=distance_to_center)
    
    def pixel_to_world_coordinates(self, px, py, z_height=0):
        """
        Convert pixel coordinates to world coordinates
        This is a simplified conversion - implement proper camera-to-world transformation
        """
        # This is a basic mapping - you'll need to calibrate this for your setup
        # Assuming working area maps to physical coordinates
        
        # Map pixel coordinates to physical coordinates (in mm)
        work_width = self.work_area['x_max'] - self.work_area['x_min']
        work_height = self.work_area['y_max'] - self.work_area['y_min']
        
        # Physical working area size (adjust for your setup)
        physical_width = 200  # mm
        physical_height = 200  # mm
        
        # Convert to relative position within working area
        rel_x = (px - self.work_area['x_min']) / work_width
        rel_y = (py - self.work_area['y_min']) / work_height
        
        # Convert to world coordinates (robot base as origin)
        world_x = rel_x * physical_width - physical_width/2
        world_y = rel_y * physical_height - physical_height/2
        world_z = z_height
        
        return world_x, world_y, world_z
    
    def calibrate_camera(self, chessboard_size=(9, 6), num_images=20):
        """
        Camera calibration using chessboard pattern
        Hold a chessboard pattern in front of camera and call this function
        """
        # Prepare object points
        objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
        
        objpoints = []  # 3D points
        imgpoints = []  # 2D points
        
        print(f"Starting calibration. Show chessboard pattern {num_images} times...")
        
        images_captured = 0
        while images_captured < num_images:
            if self.current_frame is not None:
                gray = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2GRAY)
                
                # Find chessboard corners
                ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
                
                if ret:
                    objpoints.append(objp)
                    
                    # Refine corner positions
                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                              (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                    imgpoints.append(corners2)
                    
                    # Draw corners
                    cv2.drawChessboardCorners(self.current_frame, chessboard_size, corners2, ret)
                    images_captured += 1
                    print(f"Captured image {images_captured}/{num_images}")
                    time.sleep(1)
                
                cv2.imshow('Calibration', self.current_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        
        if len(objpoints) > 0:
            # Perform calibration
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None)
            
            if ret:
                self.camera_matrix = mtx
                self.dist_coeffs = dist
                
                # Save calibration
                cal_data = {
                    'camera_matrix': mtx.tolist(),
                    'dist_coeffs': dist.tolist()
                }
                
                with open('camera_calibration.json', 'w') as f:
                    json.dump(cal_data, f, indent=2)
                
                print("Camera calibration completed and saved!")
                return True
        
        print("Calibration failed!")
        return False

class ObjectTracker:
    """Track objects across frames for stable detection"""
    
    def __init__(self, max_disappeared=30):
        self.next_object_id = 0
        self.objects = {}
        self.disappeared = {}
        self.max_disappeared = max_disappeared
    
    def register(self, centroid):
        """Register new object"""
        self.objects[self.next_object_id] = centroid
        self.disappeared[self.next_object_id] = 0
        self.next_object_id += 1
    
    def deregister(self, object_id):
        """Remove object from tracking"""
        del self.objects[object_id]
        del self.disappeared[object_id]
    
    def update(self, detections):
        """Update tracker with new detections"""
        if len(detections) == 0:
            # Mark all existing objects as disappeared
            for object_id in list(self.disappeared.keys()):
                self.disappeared[object_id] += 1
                if self.disappeared[object_id] > self.max_disappeared:
                    self.deregister(object_id)
            return self.objects
        
        # Initialize centroids array
        input_centroids = np.array([det['center'] for det in detections])
        
        if len(self.objects) == 0:
            # Register all detections as new objects
            for centroid in input_centroids:
                self.register(centroid)
        else:
            # Match existing objects to new detections
            object_centroids = list(self.objects.values())
            object_ids = list(self.objects.keys())
            
            # Compute distance matrix
            D = np.linalg.norm(np.array(object_centroids)[:, np.newaxis] - input_centroids, axis=2)
            
            # Find minimum distances
            rows = D.min(axis=1).argsort()
            cols = D.argmin(axis=1)[rows]
            
            used_row_idxs = set()
            used_col_idxs = set()
            
            # Update existing objects
            for (row, col) in zip(rows, cols):
                if row in used_row_idxs or col in used_col_idxs:
                    continue
                
                if D[row, col] > 50:  # Maximum distance threshold
                    continue
                
                object_id = object_ids[row]
                self.objects[object_id] = input_centroids[col]
                self.disappeared[object_id] = 0
                
                used_row_idxs.add(row)
                used_col_idxs.add(col)
            
            # Handle unmatched detections and objects
            unused_row_idxs = set(range(0, D.shape[0])).difference(used_row_idxs)
            unused_col_idxs = set(range(0, D.shape[1])).difference(used_col_idxs)
            
            if D.shape[0] >= D.shape[1]:
                # More objects than detections
                for row in unused_row_idxs:
                    object_id = object_ids[row]
                    self.disappeared[object_id] += 1
                    
                    if self.disappeared[object_id] > self.max_disappeared:
                        self.deregister(object_id)
            else:
                # More detections than objects
                for col in unused_col_idxs:
                    self.register(input_centroids[col])
        
        return self.objects

# Integration class to combine vision with arm control
class VisionGuidedArm:
    """Integrate vision system with robotic arm for autonomous operation"""
    
    def __init__(self, arm_controller, vision_system):
        self.arm = arm_controller
        self.vision = vision_system
        self.tracker = ObjectTracker()
        self.is_autonomous = False
        
    def start_autonomous_mode(self):
        """Start autonomous pick and place operation"""
        self.is_autonomous = True
        self.autonomous_thread = threading.Thread(target=self._autonomous_loop, daemon=True)
        self.autonomous_thread.start()
        print("Autonomous mode started")
    
    def stop_autonomous_mode(self):
        """Stop autonomous operation"""
        self.is_autonomous = False
        print("Autonomous mode stopped")
    
    def _autonomous_loop(self):
        """Main autonomous operation loop"""
        while self.is_autonomous:
            try:
                # Find target object
                target_obj = self.vision.get_largest_object('red')  # Look for red objects
                
                if target_obj:
                    print(f"Target found: {target_obj['color']} at {target_obj['center']}")
                    
                    # Convert to world coordinates
                    world_x, world_y, world_z = self.vision.pixel_to_world_coordinates(
                        target_obj['center'][0], target_obj['center'][1], 50)
                    
                    # Execute pick and place
                    self.pick_object_at_coordinates(world_x, world_y, world_z)
                    
                else:
                    print("No target objects found")
                
                time.sleep(2)  # Wait before next scan
                
            except Exception as e:
                print(f"Autonomous operation error: {e}")
                time.sleep(1)
    
    def pick_object_at_coordinates(self, x, y, z):
        """Pick object at specified world coordinates"""
        try:
            # Move arm to coordinates (you'll need to implement proper inverse kinematics)
            print(f"Moving to pick up object at ({x:.1f}, {y:.1f}, {z:.1f})")
            
            # For now, use predefined positions - implement coordinate-based movement
            self.arm.open_gripper()
            time.sleep(1)
            
            self.arm.move_to_pick_position()
            time.sleep(3)
            
            self.arm.close_gripper()
            time.sleep(2)
            
            # Move to drop location
            self.arm.move_to_drop_position()
            time.sleep(3)
            
            self.arm.open_gripper()
            time.sleep(1)
            
            self.arm.move_to_home()
            print("Pick and place completed")
            
        except Exception as e:
            print(f"Pick operation failed: {e}")

# Main application
def main():
    """Main application for vision-guided robotic arm"""
    print("Initializing Vision-Guided Robotic Arm System...")
    
    # Initialize vision system
    vision = VisionSystem(camera_id=0)
    vision.start_capture()
    
    # Import and initialize arm controller (uncomment when connected)
    # from raspberry_pi_controller import RoboticArmController
    # arm = RoboticArmController()
    # guided_arm = VisionGuidedArm(arm, vision)
    
    try:
        while True:
            print("\n=== Vision System Menu ===")
            print("1. Show live video")
            print("2. Detect objects")
            print("3. Calibrate camera")
            print("4. Set working area")
            print("5. Test object tracking")
            # print("6. Start autonomous mode")  # Uncomment when arm is connected
            print("9. Exit")
            
            choice = input("Enter choice: ").strip()
            
            if choice == '1':
                print("Showing live video. Press 'q' to quit.")
                while True:
                    if vision.current_frame is not None:
                        cv2.imshow('Live Video', vision.current_frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                    time.sleep(0.033)
                cv2.destroyAllWindows()
            
            elif choice == '2':
                print("Object detection active. Press 'q' to quit.")
                # Detection is already running in the background
                time.sleep(0.1)
                objects = vision.detected_objects
                if objects:
                    print(f"Detected {len(objects)} objects:")
                    for i, obj in enumerate(objects):
                        print(f"  {i+1}. {obj['color']} at {obj['center']} (area: {obj['area']})")
                else:
                    print("No objects detected")
            
            elif choice == '3':
                print("Starting camera calibration...")
                vision.calibrate_camera()
            
            elif choice == '4':
                print("Click and drag to set working area on the video window")
                # Implement interactive area selection
                pass
            
            elif choice == '5':
                print("Object tracking test - move objects around")
                # Test tracking functionality
                pass
            
            # elif choice == '6':
            #     guided_arm.start_autonomous_mode()
            #     input("Press Enter to stop autonomous mode...")
            #     guided_arm.stop_autonomous_mode()
            
            elif choice == '9':
                break
            
            else:
                print("Invalid choice!")
    
    except KeyboardInterrupt:
        print("\nShutting down...")
    
    finally:
        vision.stop_capture()

if __name__ == "__main__":
    main()