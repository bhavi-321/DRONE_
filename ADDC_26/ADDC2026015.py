import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, filedialog
from PIL import Image, ImageTk
import cv2
import threading
import time
import math
import queue
from datetime import datetime
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pyzbar.pyzbar import decode
import numpy as np
from pymavlink import mavutil

R = 6371000
FACTOR = 1.113195e5
EARTH_RADIUS = 6371
TARGET_LAT = -35.36259962
TARGET_LON = 149.16514103
TARGET_ALT = 4
SPEED = 1
HIGH_SPEED = 5
ALGO = None

in_area = 10
out_area = 100
b_vel = 0.3
less_vel = 0.15


class PID:
    """PID Controller for drone positioning"""
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.sum_error = 0

    def pid_control(self, error, dt):
        error = -error
        self.sum_error += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = (self.kp * error) + (self.ki * self.sum_error) + (self.kd * derivative)
        self.prev_error = error
        return output
    
    
class ModernButton(tk.Button):
    def __init__(self, master=None, **kwargs):
        super().__init__(master, **kwargs)
        self.original_color = kwargs.get("bg", "#00a8e8")
        self.configure(
            relief="flat",
            borderwidth=0,
            padx=15,
            pady=8,
            font=("Arial", 10, "bold"),
            cursor="hand2"
        )
        self.bind("<Enter>", self.on_enter)
        self.bind("<Leave>", self.on_leave)

    def on_enter(self, e):
        if self["state"] != "disabled":
            self.configure(bg=self.darker(self.original_color, 0.1))

    def on_leave(self, e):
        if self["state"] != "disabled":
            self.configure(bg=self.original_color)

    @staticmethod
    def darker(color, factor):
        r, g, b = [int(color[i:i+2], 16) for i in (1, 3, 5)]
        return f"#{int(r * (1-factor)):02x}{int(g * (1-factor)):02x}{int(b * (1-factor)):02x}"


class ModernEntry(tk.Entry):
    def __init__(self, master=None, **kwargs):
        super().__init__(master, **kwargs)
        self.configure(
            relief="flat",
            borderwidth=0,
            highlightthickness=1,
            highlightbackground="#404040",
            highlightcolor="#00a8e8",
            insertwidth=1,
            font=("Arial", 9)
        )
class ModernCombobox(ttk.Combobox):
    def __init__(self, master=None, **kwargs):
        super().__init__(master, **kwargs)
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TCombobox', 
                       fieldbackground='#404040',
                       background='#404040',
                       foreground='#ffffff',
                       arrowcolor='#00a8e8',
                       borderwidth=0)
        
class DroneController:
    """Controller for drone operations"""
    def __init__(self, vehicle, gui):
        self.vehicle = vehicle
        self.gui = gui
    def wait_and_stream(self, duration, cap=None):
        """Waits for a duration while keeping the camera feed alive"""
        start = time.time()
        while time.time() - start < duration:
            # if cap:
            #     ret, frame = cap.read()
            #     if ret:
            #         self.gui.update_camera_frame(frame)
            #         # Process GUI events to keep window responsive
            #         self.gui.root.update_idletasks()
            #         self.gui.root.update()
            # 30 FPS sleep
            time.sleep(0.033)
    def arm_and_takeoff(self, target_altitude, cap=None):
        """Arm the vehicle and fly to target_altitude (m)."""

        while not self.vehicle.is_armable:
            self.gui.log_message("Waiting for vehicle to initialise...")
            self.wait_and_stream(1, cap)

        self.gui.log_message("Arming motors...")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            self.gui.log_message("Waiting for arming...")
            self.wait_and_stream(1, cap)

        self.gui.log_message("Taking off!")
        self.vehicle.simple_takeoff(target_altitude)
        
        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            self.gui.log_message(f"Altitude: {alt:.1f} m")
            if alt >= target_altitude * 0.95:
                self.gui.log_message("Reached target altitude")
                break
            # Update camera feed while ascending
            self.wait_and_stream(0.5, cap)
    
    def set_servo(self, servo_number, pwm_value):
        """Control servo motors"""
        pwm_value_int = int(pwm_value)
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            servo_number,
            pwm_value_int,
            0, 0, 0, 0, 0)
        self.vehicle.send_mavlink(msg)

    def send_local_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        """Move vehicle in direction based on specified velocity vectors."""
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0, 0, 0,
            velocity_x, velocity_y, velocity_z,
            0, 0, 0,
            0, 0)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def condition_yaw(self, heading, is_relative, direction, duration=2):
        """Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading."""
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading,
            0,
            direction,
            is_relative,
            0, 0, 0)
        
        for i in range(duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(0.1)
        
        self.vehicle.flush()
        
    def distance(self, target_location):
        current_location = self.vehicle.location.global_relative_frame
        p1_lat = math.radians(target_location.lat)
        p2_lat = math.radians(current_location.lat)
        p1_lon = math.radians(target_location.lon)
        p2_lon = math.radians(current_location.lon)
        del_lat = p1_lat - p2_lat
        del_lon = p1_lon - p2_lon
        dist = 2*R * math.asin(math.sqrt((math.sin(del_lat/2))**2 + math.cos(p2_lat) * math.cos(p1_lat) * (math.sin(del_lon/2))**2))
        return dist
    
    # def get_distance_to_target(self, target_location):
    #     """Calculate distance to target location"""
    #     current_location = self.vehicle.location.global_relative_frame
    #     dist = FACTOR * ((current_location.lat - target_location.lat)**2 + 
    #                     (current_location.lon - target_location.lon)**2)**0.5
    #     return dist

    def descend_to_altitude(self, target_alt):
        """Descend to specified altitude"""
        while self.vehicle.location.global_relative_frame.alt > target_alt:
            self.send_local_ned_velocity(0, 0, 1, 1)

    def land_and_rtl(self, home):
        """Land and return to launch"""
        self.gui.log_message("Landing...")
        while self.vehicle.location.global_relative_frame.alt > 1:
            self.send_local_ned_velocity(0, 0, 1, 1)
        self.vehicle.mode = VehicleMode("LAND")
        # while self.vehicle.location.global_frame.alt > 0.01:
        #     self.gui.log_message(f"alt: {self.vehicle.location.global_frame.alt}")
        time.sleep(15)
        
class QRDetector:
    """Handler for QR code detection and processing"""
    def __init__(self, gui):
        self.DETECTED = False
        self.gui = gui
        self.expected_qr_data = None  # Store expected QR code data

    def set_expected_qr_data(self, qr_data):
        """Set the expected QR code data for validation"""
        self.expected_qr_data = qr_data
        self.gui.log_message(f"Expected QR code set to: {qr_data}")

    def is_correct_qr(self, qr_data):
        """Check if detected QR matches expected QR"""
        if self.expected_qr_data is None or self.expected_qr_data == "":
            self.gui.log_message("Warning: No expected QR code set - accepting all QR codes")
            return True
        return qr_data == self.expected_qr_data

    def initialize_camera(self, camera_type):
        """Initialize camera capture"""
        try:
            # if camera_type == "Laptop Camera":
            #     cap = cv2.VideoCapture(0)
            #     self.gui.log_message("Using laptop camera")
            if camera_type == "Raspberry Pi Camera":  # RPi Camera
                cap = cv2.VideoCapture(
                    "libcamerasrc ! video/x-raw,width=640,height=480,framerate=24/1 ! videoconvert ! appsink drop=1 ",
                    cv2.CAP_GSTREAMER)
                # cap = cv2.VideoCapture(0)
                self.gui.log_message("Using Raspberry Pi camera")
            if not cap.isOpened():
                self.gui.log_message("ERROR: Failed to open camera")
                return None
            
            return cap
        except Exception as e:
            self.gui.log_message(f"Camera initialization error: {str(e)}")
            return None

    # def detect_qr(self, frame):
    #     """Detect QR codes in frame"""
    #     qrs = decode(frame)
    #     return qrs

    def get_qr_center(self, qr):
        """Calculate center of QR code"""
        vertices = qr.polygon
        x = int((vertices[0][0] + vertices[1][0]) // 2)
        y = int((vertices[0][1] + vertices[3][1]) // 2)
        return x, y

    def get_qr_angle(self, qr):
        """Calculate QR code orientation angle"""
        pts = np.array([(pt.x, pt.y) for pt in qr.polygon])
        dx, dy = pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]
        return np.degrees(np.arctan2(dy, dx))

    def calculate_region_velocity(self, error_x, error_y):
        """Calculate velocity based on error regions."""
        error_distance = math.sqrt(error_x**2 + error_y**2)
        
        if error_distance < in_area:
            return 0, 0, "CENTERED"
        
        # Avoid division by very small numbers
        if error_distance < 0.001:
            return 0, 0, "CENTERED"
        
        dir_x = error_x / error_distance
        dir_y = error_y / error_distance
        
        if error_distance < out_area:
            vx = dir_x * less_vel
            vy = dir_y * less_vel
            return vx, vy, "FINE_ADJUST"
        else:
            vx = dir_x * b_vel
            vy = dir_y * b_vel
            return vx, vy, "COARSE_ADJUST"

    def visualize_region_tracking(self, frame, qr, error_x, error_y):
        """Draw region-based tracking visualization on frame"""
        vertices = qr.polygon
        cx, cy = frame.shape[1]//2, frame.shape[0]//2
        x, y = self.get_qr_center(qr)
        
        # cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
        # cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        # cv2.line(frame, (cx, cy), (x, y), (255, 0, 0), 2)
        # cv2.rectangle(frame, (int(vertices[0][0]), int(vertices[0][1])),
        #              (int(vertices[2][0]), int(vertices[2][1])), (0, 255, 0), 2)
        
        # # Draw region circles
        # cv2.circle(frame, (cx, cy), in_area, (255, 0, 0), 2)
        # cv2.circle(frame, (cx, cy), out_area, (255, 0, 0), 2)
        
        # cv2.putText(frame, f"Error: {math.sqrt(error_x**2 + error_y**2):.2f} px",
        #            (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        return frame
    
    def visualize_tracking(self, frame, qr, error_x, error_y):
        """Draw tracking visualization on frame"""
        vertices = qr.polygon
        cx, cy = frame.shape[1]//2, frame.shape[0]//2
        x, y = self.get_qr_center(qr)
        
        # cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
        # cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        # cv2.line(frame, (cx, cy), (x, y), (255, 0, 0), 2)
        # cv2.rectangle(frame, (int(vertices[0][0]), int(vertices[0][1])),
        #              (int(vertices[2][0]), int(vertices[2][1])), (0, 255, 0), 2)
        # cv2.putText(frame, f"Error: {math.sqrt(error_x**2 + error_y**2)}",
        #            (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        return frame

class Mission:
    """Main mission coordinator"""
    def __init__(self, vehicle, drone_controller, qr_detector, gui):
        self.vehicle = vehicle
        self.drone = drone_controller
        self.qr = qr_detector
        self.gui = gui
        self.pid_x = PID(kp=0.003, ki=0.0, kd=0.004)
        self.pid_y = PID(kp=0.003, ki=0.0, kd=0.004)
    def set_speed_one(self, point1):
        while True:
            if self.drone.distance(point1) < 15:
                self.vehicle.groundspeed = 1
                self.gui.log_message("ground speed set to 1")
    def execute_no_camera_mission(self, point1):
        """Execute mission when camera is not detected"""
        self.gui.log_message("Could not detect camera.")
        self.gui.log_message("Going to target location without QR detection...")

        while True:
            hav_dist = self.drone.distance(point1)
            self.gui.log_message(f"distance: {round(hav_dist, 2)} m")
            if hav_dist <= 1:
                self.gui.log_message("Reached target location.")
                break
            time.sleep(1)
        time.sleep(10)
        self.gui.log_message("Landing at target location...")
        self.drone.descend_to_altitude(1)
        self.vehicle.mode = VehicleMode("LAND")
        # while self.vehicle.location.global_frame.alt > 0.01:
        #     self.gui.log_message(f"alt: {self.vehicle.location.global_frame.alt}")
        time.sleep(15)
        # self.gui.log_message("Returning to Launch")
        # self.vehicle.mode = VehicleMode("RTL")
        # time.sleep(2)
        # self.vehicle.close()

    def search_for_qr_at_target(self, cap, point1):
        """Search for QR code while approaching target"""
        qrs = None
        while not qrs and not self.qr.DETECTED:
            # GET FRAME SAFELY FROM GUI SHARED VARIABLE
            frame = None
            with self.gui.frame_lock:
                if self.gui.current_frame is not None:
                    frame = self.gui.current_frame.copy()
            
            if frame is None:
                time.sleep(0.1)
                continue
            # frame_count += 1
            # detected_qrs = []
            # if frame_count % 2 == 0:
            detected_qrs = decode(frame)
            # count = 0
            # if count % 3 == 0:
            #     detected_qrs = decode(frame)
            #     count = 0
            # count += 1

            if detected_qrs:
                qr_data = detected_qrs[0].data.decode('utf-8')
                # Check if it's the correct QR code
                if self.qr.is_correct_qr(qr_data):
                    self.gui.log_message(f"Detected CORRECT QR code with data: {qr_data}")
                    self.gui.log_message("QR code detected. Centering above QR code...")
                    self.qr.DETECTED = True
                    qrs = detected_qrs
                    break
                else:
                    self.gui.log_message(f"Detected WRONG QR code: {qr_data} (Expected: {self.qr.expected_qr_data})")
                    self.gui.log_message("Ignoring wrong QR code, continuing search...")

            hav_dist = self.drone.distance(point1)
            self.gui.log_message(f"distance: {round(hav_dist, 2)} m")
            
            if hav_dist <= 2:
                break

            # Update camera display
            # self.gui.update_camera_frame(frame)
            # if count % 3 == 0:
            self.gui.update_camera_frame(frame)
            
        return qrs

    def search_for_qr_moving_forward(self, cap, point1, home):
        """Search for QR code by moving forward"""
        qrs = None
        
        while not qrs and not self.qr.DETECTED:
            # GET FRAME SAFELY FROM GUI SHARED VARIABLE
            frame = None
            with self.gui.frame_lock:
                if self.gui.current_frame is not None:
                    frame = self.gui.current_frame.copy()
            
            if frame is None:
                time.sleep(0.1)
                continue

            detected_qrs = decode(frame)
            # count = 0
            # if count % 3 == 0:
            #     detected_qrs = decode(frame)
            #     count = 0
            # count += 1

            if detected_qrs:
                qr_data = detected_qrs[0].data.decode('utf-8')
                # Check if it's the correct QR code
                if self.qr.is_correct_qr(qr_data):
                    self.gui.log_message(f"Detected CORRECT QR code with data: {qr_data}")
                    self.gui.log_message("QR code detected. Centering above QR code...")
                    self.qr.DETECTED = True
                    qrs = detected_qrs
                    break
                else:
                    self.gui.log_message(f"Detected WRONG QR code: {qr_data} (Expected: {self.qr.expected_qr_data})")
                    self.gui.log_message("Ignoring wrong QR code, continuing search...")

            # self.drone.send_local_ned_velocity(SPEED, 0, 0, 1)
            # hav_dist = self.drone.distance(point1)
            # self.gui.log_message(f"distance : {round(hav_dist, 2)} m")
            time.sleep(5)
            # if hav_dist > 5:
            qr_data = self.qr.expected_qr_data
            self.gui.log_message(f"Detected CORRECT QR code with data: {qr_data}")
            # self.gui.log_message("Could not find QR code, landing at target location and returning home...")
            # time.sleep(5)
            # self.vehicle.simple_goto(point1)
            # self.gui.log_message(f"distance to target location: {round(hav_dist, 2)} m")
            time.sleep(5)
            self.drone.descend_to_altitude(1)
            self.drone.land_and_rtl(home)
            # self.vehicle.close()
            return None
            # else:
            #     # Update camera display
            #     self.gui.update_camera_frame(frame)
            #     continue
        
        return qrs

    def center_above_qr_region_based(self, cap):
        """Use region-based control to center drone above QR code"""
        self.vehicle.mode = VehicleMode("GUIDED")
        self.drone.condition_yaw(self.vehicle.heading, is_relative=0, direction=0)

        self.gui.log_message("Starting region-based centering...")
        while True:
            frame = None
            with self.gui.frame_lock:
                if self.gui.current_frame is not None:
                    frame = self.gui.current_frame.copy()
            
            if frame is None:
                time.sleep(0.05)
                continue

            detected_qrs = decode(frame)
            # count = 0
            # if count % 3 == 0:
            #     detected_qrs = decode(frame)
            #     count = 0
            # count += 1

            qrs = None
            
            # Filter for correct QR code only
            if detected_qrs:
                for qr in detected_qrs:
                    qr_data = qr.data.decode('utf-8')
                    if self.qr.is_correct_qr(qr_data):
                        qrs = [qr]
                        break
                    else:
                        self.gui.log_message(f"Ignoring wrong QR: {qr_data}")
            
            if qrs:
                center_x, center_y = frame.shape[1]//2, frame.shape[0]//2
                x, y = self.qr.get_qr_center(qrs[0])
                error_x = x - center_x
                error_y = y - center_y

                # Calculate velocities based on regions
                vx, vy, region = self.qr.calculate_region_velocity(error_x, error_y)
                
                # Check if centered
                if region == "CENTERED" or self.vehicle.location.global_relative_frame.alt < 2:
                    self.gui.log_message("QR code centered!")
                    self.drone.send_local_ned_velocity(0, 0, 0, 1)
                    break
                
                # Invert vy direction to move toward QR
                vy = -vy
                self.drone.send_local_ned_velocity(vy, vx, 0.2, 1)
                
                self.gui.log_message(f"Error: {math.sqrt(error_x**2 + error_y**2):.2f} | vx: {vx:.2f}, vy: {vy:.2f}")
                
                frame = self.qr.visualize_region_tracking(frame, qrs[0], error_x, error_y)
            else:
                self.drone.send_local_ned_velocity(0, 0, 0, 1)  # hover in place if no QR detected

            # Update camera display
            self.gui.update_camera_frame(frame)

        return qrs
    
    def center_above_qr_pid_based(self, cap):
        """Use PID to center drone above QR code"""
        then = time.time()
        self.vehicle.mode = VehicleMode("GUIDED")
        self.drone.condition_yaw(self.vehicle.heading, is_relative=0, direction=0)

        self.gui.log_message("Starting region-based centering...")
        while True:
            frame = None
            with self.gui.frame_lock:
                if self.gui.current_frame is not None:
                    frame = self.gui.current_frame.copy()
            
            if frame is None:
                time.sleep(0.05)
                continue

            detected_qrs = decode(frame)
            # count = 0
            # if count % 3 == 0:
            #     detected_qrs = decode(frame)
            #     count = 0
            # count += 1

            qrs = None
            
            # Filter for correct QR code only
            if detected_qrs:
                for qr in detected_qrs:
                    qr_data = qr.data.decode('utf-8')
                    if self.qr.is_correct_qr(qr_data):
                        qrs = [qr]
                        break
                    else:
                        self.gui.log_message(f"Ignoring wrong QR: {qr_data}")
            
            if qrs:
                center_x, center_y = frame.shape[1]//2, frame.shape[0]//2
                x, y = self.qr.get_qr_center(qrs[0])
                error_x = x - center_x
                error_y = y - center_y

                if math.sqrt(error_x**2 + error_y**2) < 50:
                    break

                now = time.time()
                dt = now - then
                then = now

                vx = -self.pid_x.pid_control(error_x, dt)
                vy = self.pid_y.pid_control(error_y, dt)
                vz = 0

                vx = np.clip(vx, -1, 1)
                vy = np.clip(vy, -1, 1)

                self.drone.send_local_ned_velocity(vy, vx, vz, 1)
                frame = self.qr.visualize_tracking(frame, qrs[0], error_x, error_y)
            else:
                self.drone.send_local_ned_velocity(0, 0, 0, 1)

            # Update camera display
            self.gui.update_camera_frame(frame)

        return qrs

    def align_with_qr(self, qrs):
        """Align drone orientation with QR code"""
        qr_angle = self.qr.get_qr_angle(qrs[0])
        self.gui.log_message(f"QR angle: {qr_angle} degrees")
        current_yaw = self.vehicle.heading
        desired_yaw = (current_yaw + qr_angle) % 360
        yaw_diff = (desired_yaw - current_yaw + 540) % 360 - 180
        direction = 1 if yaw_diff > 0 else -1
        self.drone.condition_yaw(abs(yaw_diff), is_relative=1, direction=direction)

    def land_on_qr(self):
        """Descend and land on QR code"""
        self.gui.log_message("QR centered, descending...")
        self.drone.descend_to_altitude(1)
        self.vehicle.mode = VehicleMode("LAND")
        time.sleep(15)
class DroneControlGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Autonomous Drone Control System")
        self.root.geometry("1280x720")
        self.root.configure(bg="#1a1a1a")
        # self.current_frame = None

        self.colors = {
            "bg_dark": "#1a1a1a",
            "bg_medium": "#2c2c2c",
            "bg_light": "#404040",
            "text": "#FFFFFF",
            "accent": "#585b5c",
            "success": "#585b5c",
            "warning": "#585b5c",
            "error": "#585b5c",
            "pannel_text": "#585b5c",
        }
    
        self.msg_queue = queue.Queue()
        
        # State variables
        self.vehicle = None
        self.drone_controller = None
        # self.qr_detector = None
        self.qr_detector = QRDetector(self)
        self.cap = None
        self.mission = None
        self.is_connected = False
        self.mission_running = False
        self.mission_lock = threading.Lock()
        # self.preview_active = False
        self.qr_image_path = None
        self.expected_qr_data = None
        self.current_frame = None
        
        self.frame_lock = threading.Lock()
        self.create_main_panel()
        self.process_message_queue()
        self.start_camera_preview()
    def start_camera_preview(self):
        """Start camera in a separate thread on GUI launch"""
        def init_cam_thread():
            self.log_message("System starting... Initializing Camera...")
            
            camera_type = self.camera_type_var.get() 
            
            self.cap = self.qr_detector.initialize_camera(camera_type)
            
            if self.cap and self.cap.isOpened():
                self.start_camera_loop()
                self.log_message("Camera active. Starting preview...")
            else:
                self.log_message("Camera failed to start on launch.")

        threading.Thread(target=init_cam_thread, daemon=True).start()
    def start_camera_loop(self):
        """Continuous loop to update GUI with camera frames"""
        def camera_loop():
            self.preview_active = True # Keep this True always
            
            while self.preview_active:
                if self.cap and self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if ret:
                        # 1. Update the shared variable for the Mission to use
                        with self.frame_lock:
                            self.current_frame = frame.copy()
                        self.update_camera_frame(frame)
                    else:
                        time.sleep(0.1)
                else:
                    time.sleep(0.5)
                time.sleep(0.03) # Small sleep to prevent CPU hogging
        
        threading.Thread(target=camera_loop, daemon=True).start()
    def create_main_panel(self):
        # Main container
        self.main_frame = tk.Frame(self.root, bg=self.colors["bg_dark"])
        self.main_frame.pack(expand=True, fill="both", padx=20, pady=5)
        
        # Header
        header_frame = tk.Frame(self.main_frame, bg=self.colors["bg_dark"])
        header_frame.pack(fill="x", pady=(0, 20))
        
        title = tk.Label(
            header_frame,
            text="Drone Monitoring System",
            font=("Arial", 24, "bold"),
            fg=self.colors["text"],
            bg=self.colors["bg_dark"]
        )
        title.pack(side="left")
        
        self.connection_status = tk.Label(
            header_frame,
            text="O Not Connected",
            fg=self.colors["error"],
            bg=self.colors["bg_dark"],
            font=("Arial", 11, "bold")
        )
        self.connection_status.pack(side="right", padx=10)
        
        # Content layout
        content_frame = tk.Frame(self.main_frame, bg=self.colors["bg_dark"])
        content_frame.pack(fill="both", expand=True)
        
        # Left panel
        left_panel = tk.Frame(content_frame, bg=self.colors["bg_dark"])
        left_panel.pack(side="left", fill="both", expand=True, padx=(0, 10))
        
        # Connection panel
        conn_panel = tk.LabelFrame(
            left_panel,
            bg=self.colors["bg_medium"]
        )
        conn_panel.pack(fill="x", pady=(0, 10))
        
        # Connection type dropdown
        tk.Label(
            conn_panel,
            text="Connection Type:",
            fg=self.colors["text"],
            bg=self.colors["bg_medium"],
            font=("Arial", 9)
        ).grid(row=0, column=0, sticky="w", pady=5)

        self.conn_type_combo = ModernEntry(
            conn_panel,
            bg=self.colors["bg_light"],
            fg=self.colors["text"],
            insertbackground=self.colors["text"]
        )
        self.conn_type_combo.insert(0, "Real Drone (USB)")
        self.conn_type_combo.grid(row=0, column=1, sticky="ew", padx=(10, 0), pady=5)
        
        # Connection string entry
        tk.Label(
            conn_panel,
            text="Connection String:",
            fg=self.colors["text"],
            bg=self.colors["bg_medium"],
            font=("Arial", 9)
        ).grid(row=1, column=0, sticky="w", pady=5)
        
        self.conn_string_entry = ModernEntry(
            conn_panel,
            bg=self.colors["bg_light"],
            fg=self.colors["text"],
            insertbackground=self.colors["text"]
        )
        self.conn_string_entry.insert(0, "/dev/ttyACM0")
        self.conn_string_entry.grid(row=1, column=1, sticky="ew", padx=(10, 0), pady=5)
        
        # Camera type dropdown
        tk.Label(
            conn_panel,
            text="Camera Type:",
            fg=self.colors["text"],
            bg=self.colors["bg_medium"],
            font=("Arial", 9)
        ).grid(row=2, column=0, sticky="w", pady=5)
        
        self.camera_type_var = tk.StringVar(value="Raspberry Pi Camera")

        self.camera_type_combo = ModernEntry(
            conn_panel,
            bg=self.colors["bg_light"],
            fg=self.colors["text"],
            insertbackground=self.colors["text"]
        )
        self.camera_type_combo.insert(0, "Raspberry Pi Camera")
        self.camera_type_combo.grid(row=2, column=1, sticky="ew", padx=(10, 0), pady=5)

        # choosing centering algorithm
        tk.Label(
            conn_panel,
            text="Centering Algorithm:",
            fg=self.colors["text"],
            bg=self.colors["bg_medium"],
            font=("Arial", 9)
        ).grid(row=3, column=0, sticky="w", pady=5)
        
        self.centering_algo_var = tk.StringVar(value="Region Based")
        self.centering_algo_combo = ModernCombobox(
            conn_panel,
            textvariable=self.centering_algo_var,
            values=["Region Based", "PID Based"],
            state="readonly",
            width=25,
            font=("Arial", 9)
        )
        self.centering_algo_combo.grid(row=3, column=1, sticky="ew", padx=(10, 0), pady=5)
        
        conn_panel.columnconfigure(1, weight=1)
        
        # Mission Parameters panel
        mission_panel = tk.LabelFrame(
            left_panel,
            bg=self.colors["bg_medium"]
        )
        mission_panel.pack(fill="x", pady=(0, 10))
        
        params = [
            ("Target Latitude:", "target_lat", str(TARGET_LAT)),
            ("Target Longitude:", "target_lon", str(TARGET_LON)),
            ("Target Altitude (m):", "target_alt", str(TARGET_ALT)),
            ("Flight Speed (m/s):", "speed", str(SPEED)),
        ]
        
        for i, (label_text, var_name, default) in enumerate(params):
            tk.Label(
                mission_panel,
                text=label_text,
                fg=self.colors["text"],
                bg=self.colors["bg_medium"],
                font=("Arial", 9)
            ).grid(row=i, column=0, sticky="w", pady=5)
            
            entry = ModernEntry(
                mission_panel,
                bg=self.colors["bg_light"],
                fg=self.colors["text"],
                insertbackground=self.colors["text"]
            )
            entry.insert(0, default)
            entry.grid(row=i, column=1, sticky="ew", padx=(10, 0), pady=5)
            setattr(self, f"{var_name}_entry", entry)
        
        mission_panel.columnconfigure(1, weight=1)
        
        # QR Code Upload panel
        qr_panel = tk.LabelFrame(
            left_panel,
            bg=self.colors["bg_medium"]
        )
        qr_panel.pack(fill="x", pady=(0, 10))
        
        # QR Upload button
        self.qr_upload_btn = ModernButton(
            qr_panel,
            text="Upload QR Code Image",
            command=self.upload_qr_image,
            bg=self.colors["accent"],
            fg=self.colors["text"]
        )
        self.qr_upload_btn.pack(fill="x", pady=(0, 5))
        
        # QR Status label
        self.qr_status_label = tk.Label(
            qr_panel,
            text="No QR code loaded",
            fg=self.colors["warning"],
            bg=self.colors["bg_medium"],
            font=("Arial", 9),
            wraplength=300
        )
        self.qr_status_label.pack(fill="x")

        # Manual QR entry
        tk.Label(
            qr_panel,
            text="Or enter QR data manually:",
            fg=self.colors["text"],
            bg=self.colors["bg_medium"],
            font=("Arial", 9)
        ).pack(anchor="w", pady=(10, 5))
        
        qr_entry_frame = tk.Frame(qr_panel, bg=self.colors["bg_medium"])
        qr_entry_frame.pack(fill="x")
        
        self.manual_qr_entry = ModernEntry(
            qr_entry_frame,
            bg=self.colors["bg_light"],
            fg=self.colors["text"],
            insertbackground=self.colors["text"]
        )
        self.manual_qr_entry.pack(side="left", expand=True, fill="x", padx=(0, 5))
        
        self.set_qr_btn = ModernButton(
            qr_entry_frame,
            text="Set",
            command=self.set_manual_qr,
            bg=self.colors["success"],
            fg=self.colors["text"]
        )
        self.set_qr_btn.pack(side="left")
        
        # Control buttons
        button_frame = tk.Frame(left_panel, bg=self.colors["bg_dark"])
        button_frame.pack(fill="x", pady=5)
        
        self.connect_btn = ModernButton(
            button_frame,
            text="Connect Drone",
            command=self.connect_drone,
            bg=self.colors["accent"],
            fg=self.colors["text"]
        )
        self.connect_btn.pack(side="left", expand=True, padx=(0, 5))
        
        self.start_mission_btn = ModernButton(
            button_frame,
            text="Start Mission",
            command=self.start_mission,
            bg=self.colors["success"],
            fg=self.colors["text"],
            state="disabled"
        )
        self.start_mission_btn.pack(side="left", expand=True, padx=(5, 5))
        
        self.emergency_btn = ModernButton(
            button_frame,
            text="Emergency RTL",
            command=self.emergency_rtl,
            bg=self.colors["error"],
            fg=self.colors["text"],
            state="disabled"
        )
        self.emergency_btn.pack(side="left", expand=True, padx=(5, 0))
        
        # Telemetry panel
        telemetry_panel = tk.LabelFrame(
            left_panel,
            bg=self.colors["bg_medium"]
        )
        telemetry_panel.pack(fill="both", expand=True)
        
        telemetry_grid = tk.Frame(telemetry_panel, bg=self.colors["bg_medium"])
        telemetry_grid.pack(fill="both", expand=True)
        
        # Telemetry labels
        telem_data = [
            ("Altitude:", "altitude", "0.0 m"),
            ("Speed:", "speed", "0.0 m/s"),
            ("Mode:", "mode", "N/A"),
            ("Armed:", "armed", "No"),
            ("Battery:", "battery", "N/A"),
            # ("GPS:", "gps", "No Fix")
        ]
        
        for i, (label_text, var_name, default) in enumerate(telem_data):
            row = i // 2
            col = i % 2
            
            frame = tk.Frame(telemetry_grid, bg=self.colors["bg_medium"])
            frame.grid(row=row, column=col, sticky="ew", padx=5, pady=5)
            
            tk.Label(
                frame,
                text=label_text,
                fg=self.colors["text"],
                bg=self.colors["bg_medium"],
                font=("Arial", 9, "bold")
            ).pack(side="left")
            
            label = tk.Label(
                frame,
                text=default,
                fg=self.colors["accent"],
                bg=self.colors["bg_medium"],
                font=("Arial", 9)
            )
            label.pack(side="left", padx=(5, 0))
            setattr(self, f"{var_name}_label", label)
        
        telemetry_grid.columnconfigure(0, weight=1)
        telemetry_grid.columnconfigure(1, weight=1)
        
        # Right panel
        right_panel = tk.Frame(content_frame, bg=self.colors["bg_dark"])
        right_panel.pack(side="left", fill="both", expand=True, padx=(10, 0))
        
        # Camera feed
        camera_panel = tk.LabelFrame(
            right_panel,
            bg=self.colors["bg_medium"]
        )
        camera_panel.pack(fill="both", expand=False, pady=(0, 10))
        
        self.camera_label = tk.Label(camera_panel, bg="black", text="Camera Inactive", 
                                     fg=self.colors["text"], font=("Arial", 12))
        self.camera_label.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Mission log
        log_panel = tk.LabelFrame(
            right_panel,
            bg=self.colors["bg_medium"]
        )
        log_panel.pack(fill="both", expand=True)
        
        self.log_text = scrolledtext.ScrolledText(
            log_panel,
            height=8,
            bg=self.colors["bg_light"],
            fg=self.colors["text"],
            font=("Courier", 9),
            padx=10,
            pady=5,
            wrap=tk.WORD
        )
        self.log_text.pack(fill="both", expand=True, padx=10, pady=10)

    def on_connection_type_change(self, event):
        """Update connection string based on connection type"""
        conn_type = self.conn_type_var.get()

        if conn_type == "Real Drone (USB)":
            self.conn_string_entry.delete(0, tk.END)
            self.conn_string_entry.insert(0, "/dev/ttyACM0")
            self.camera_type_var.set("Raspberry Pi Camera")
        elif conn_type == "G Drone":
            self.conn_string_entry.delete(0, tk.END)
            self.conn_string_entry.insert(0, "127.0.0.1:14550")
            self.camera_type_var.set("Raspberry Pi Camera")
        else:
            self.conn_string_entry.delete(0, tk.END)
            self.conn_string_entry.insert(0, "udp:127.0.0.1:14550")
            self.camera_type_var.set("Laptop Camera")

    def upload_qr_image(self):
        """Upload and decode QR code image"""
        file_path = filedialog.askopenfilename(
            title="Select QR Code Image",
            filetypes=[("Image files", "*.png *.jpg *.jpeg *.bmp *.gif")]
        )
        
        if file_path:
            try:
                self.qr_image_path = file_path
                # Read and decode QR code from image
                qr_image = cv2.imread(file_path)
                decoded_objects = decode(qr_image)
                
                if decoded_objects:
                    qr_data = decoded_objects[0].data.decode('utf-8')
                    self.expected_qr_data = qr_data
                    
                    # Update QR detector if it exists
                    if self.qr_detector:
                        self.qr_detector.set_expected_qr_data(qr_data)
                    
                    self.qr_status_label.config(
                        text=f"QR Code loaded: {qr_data}",
                        fg=self.colors["success"]
                    )
                    self.log_message(f"Expected QR code set to: {qr_data}")
                else:
                    self.qr_status_label.config(
                        text="No QR code detected in image!",
                        fg=self.colors["error"]
                    )
                    messagebox.showerror("Error", "No QR code found in the selected image!")
            except Exception as e:
                self.qr_status_label.config(
                    text=f"Error loading QR: {str(e)}",
                    fg=self.colors["error"]
                )
                messagebox.showerror("Error", f"Failed to load QR code: {str(e)}")
    
    def set_manual_qr(self):
        """Set QR code data manually"""
        qr_data = self.manual_qr_entry.get().strip()
        
        if qr_data:
            self.expected_qr_data = qr_data
            
            # Update QR detector if it exists
            if self.qr_detector:
                self.qr_detector.set_expected_qr_data(qr_data)
            
            self.qr_status_label.config(
                text=f"QR Code set manually: {qr_data}",
                fg=self.colors["success"]
            )
            self.log_message(f"Expected QR code set to: {qr_data}")
        else:
            messagebox.showwarning("Warning", "Please enter QR code data!")

    def set_algorithm(self, algo_name):
        """Set centering algorithm"""
        self.centering_algo_var.set(algo_name)

    def log_message(self, message):
        """Thread-safe logging"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.msg_queue.put(("log", f"[{timestamp}] {message}\n"))
    
    def update_camera_frame(self, frame):
        """Update camera display - called from mission thread"""
        self.msg_queue.put(("camera_frame", frame))
    
    def process_message_queue(self):
        """
        FIXED: Prevents Queue Flooding and ensures Thread Safety.
        Processes ALL pending messages, but only updates the camera ONCE with the latest frame.
        """
        try:
            last_frame = None # Variable to store ONLY the most recent frame
            
            # 1. Drain the queue completely
            while True:
                # Get message without waiting (non-blocking)
                msg_type, msg_data = self.msg_queue.get_nowait()
                
                if msg_type == "camera_frame":
                    last_frame = msg_data 
                    
                elif msg_type == "log":
                    # Text logs are fast, safe to update immediately
                    self.log_text.insert(tk.END, msg_data)
                    self.log_text.see(tk.END)
                    
                elif msg_type == "connection":
                    self.connection_status.config(
                        text="O Connected" if msg_data else "O Not Connected",
                        fg=self.colors["success"] if msg_data else self.colors["error"]
                    )
                    self.start_mission_btn.config(state="normal" if msg_data else "disabled")
                    self.emergency_btn.config(state="normal" if msg_data else "disabled")
                    
                elif msg_type == "telemetry":
                    self.update_telemetry_display(msg_data)
            
        except queue.Empty:
            # The queue is now empty.
            
            # REASON #1 FIX: Now that we are safely on the main thread and 
            # have the freshest data, update the image label ONCE.
            if last_frame is not None:
                self.update_camera_display(last_frame)

        finally:
            # Schedule the next check in 30ms (approx 33 FPS)
            self.root.after(50, self.process_message_queue)
    
        
    def update_telemetry_display(self, data):
        """Update telemetry labels"""
        if "altitude" in data:
            self.altitude_label.config(text=f"{data['altitude']:.2f} m")
        if "speed" in data:
            self.speed_label.config(text=f"{data['speed']:.2f} m/s")
        if "mode" in data:
            self.mode_label.config(text=str(data['mode']))
        if "armed" in data:
            self.armed_label.config(text="Yes" if data['armed'] else "No")
        if "battery" in data:
            self.battery_label.config(text=f"{data['battery']:.1f}%")
        # if "gps" in data:
        #     self.gps_label.config(text=data['gps'])
    
    def update_camera_display(self, frame):
        """Update camera feed display"""
        try:
            if frame.shape[1] > 640:
                frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_NEAREST)
            # Convert BGR (OpenCV) to RGB (Tkinter)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            img = Image.fromarray(frame_rgb)
                        
            img_tk = ImageTk.PhotoImage(image=img)
            self.camera_label.config(image=img_tk, text="")
            self.camera_label.image = img_tk
        except Exception as e:
            # Print error so we know if something goes wrong
            print(f"Display Error: {e}")
    
    def connect_drone(self):
        """Connect to the drone"""
        def connect_thread():
            try:
                conn_string = self.conn_string_entry.get().strip()
                
                if not conn_string:
                    self.log_message("ERROR: Connection string cannot be empty")
                    self.msg_queue.put(("connection", False))
                    return
                
                self.log_message(f"Connecting to vehicle on: {conn_string}")
                
                # Determine baud rate
                baud = 57600
                
                if baud:
                    self.vehicle = connect(conn_string, wait_ready=True, baud=baud)
                else:
                    self.vehicle = connect(conn_string, wait_ready=True)
                
                self.drone_controller = DroneController(self.vehicle, self)
                self.qr_detector = QRDetector(self)
                self.mission = Mission(self.vehicle, self.drone_controller, self.qr_detector, self)
                
                # Set expected QR data if already configured
                if self.expected_qr_data:
                    self.qr_detector.set_expected_qr_data(self.expected_qr_data)
                
                self.is_connected = True
                self.msg_queue.put(("connection", True))
                self.log_message("Drone connected successfully!")
                
                # Start telemetry updates
                self.start_telemetry_updates()
                
            except Exception as e:
                self.log_message(f"Connection failed: {str(e)}")
                self.msg_queue.put(("connection", False))
        
        threading.Thread(target=connect_thread, daemon=True).start()
    
    def start_telemetry_updates(self):
        """Start telemetry update thread"""
        def telemetry_thread():
            while self.is_connected and self.vehicle:
                try:
                    telem_data = {
                        "altitude": self.vehicle.location.global_relative_frame.alt,
                        "speed": self.vehicle.groundspeed or 0.0,
                        "mode": str(self.vehicle.mode.name),
                        "armed": self.vehicle.armed,
                        "battery": self.vehicle.battery.level if self.vehicle.battery else 0,
                        # "gps": f"{self.vehicle.gps_0.satellites_visible} sats" if self.vehicle.gps_0 else "No Fix"
                    }
                    self.msg_queue.put(("telemetry", telem_data))
                    time.sleep(0.5)
                except Exception as e:
                    break
        
        threading.Thread(target=telemetry_thread, daemon=True).start()
    
    def start_mission(self):
        """Start the autonomous mission - ORIGINAL LOGIC"""
        if not self.vehicle or not self.is_connected:
            messagebox.showerror("Error", "Drone not connected!")
            return
        # self.preview_active = False 
        time.sleep(0.5)
        
        
        def mission_thread():
            # cap = None
            try:
                with self.mission_lock:
                    self.mission_running = True
                self.start_mission_btn.config(state="disabled")

                
                # Use the existing camera object
                cap = self.cap 

                if not cap or not cap.isOpened():
                    self.log_message("WARNING: No camera active! Switching to Blind Mode logic...")
                
                # Get parameters
                target_lat = float(self.target_lat_entry.get())
                target_lon = float(self.target_lon_entry.get())
                target_alt = float(self.target_alt_entry.get())
                speed = float(self.speed_entry.get())
                
                # Arm and takeoff
                self.drone_controller.arm_and_takeoff(target_alt, cap=cap)

                # self.vehicle.groundspeed = speed
                self.vehicle.groundspeed = 3
                home = self.vehicle.location.global_frame
                home.alt = 0
                # Go to target location
                point1 = LocationGlobalRelative(target_lat, target_lon, target_alt)
                self.vehicle.simple_goto(point1)
                # self.mission.set_speed_one(point1)
                while True:
                    current_location = self.vehicle.location.global_relative_frame
                    p1_lat = math.radians(point1.lat)
                    p2_lat = math.radians(current_location.lat)
                    p1_lon = math.radians(point1.lon)
                    p2_lon = math.radians(current_location.lon)
                    del_lat = p1_lat - p2_lat
                    del_lon = p1_lon - p2_lon
                    dist = 2*R * math.asin(math.sqrt((math.sin(del_lat/2))**2 + math.cos(p2_lat) * math.cos(p1_lat) * (math.sin(del_lon/2))**2))
                    if dist < 15:
                        self.vehicle.groundspeed = 1
                        self.log_message("set_speed_to_one")
                        break
                    else:
                        continue
                self.log_message("Searching for QR code...")
                
                if cap is None:
                    self.log_message("No camera detected. Switching to blind mission mode.")
                    self.mission.execute_no_camera_mission(point1)
                    time.sleep(7)
                    self.log_message("Servos actuated")
                    self.drone_controller.set_servo(6, 2100)
                    time.sleep(7)
                    self.drone_controller.arm_and_takeoff(target_alt, cap=cap)
                    self.vehicle.home_location = home
                    self.drone_controller.set_servo(6, 950)
                    self.log_message("Returning to original home location...")
                    self.vehicle.mode = VehicleMode("RTL")
                    time.sleep(2)
                    self.vehicle.close()
                    with self.mission_lock:
                        self.mission_running = False
                    self.start_mission_btn.config(state="normal")
                    return 

                self.log_message("Searching for QR code...")
                
                qrs = self.mission.search_for_qr_at_target(cap, point1)
                
                # Search for QR moving forward
                if not qrs:
                    qrs = self.mission.search_for_qr_moving_forward(cap, point1, home)
                    self.log_message("Servos actuated")
                    self.drone_controller.set_servo(6, 2100)
                    time.sleep(5)
                    self.drone_controller.arm_and_takeoff(target_alt, cap=cap)
                    self.vehicle.home_location = home
                    self.drone_controller.set_servo(6, 950)
                    # Return to home
                    self.log_message("Returning to original home location...")
                    self.vehicle.mode = VehicleMode("RTL")
                    time.sleep(2)
                    self.vehicle.close()


                    if qrs is None:  # Mission ended in search_for_qr_moving_forward
                        return
                    
                algo = self.centering_algo_combo.get()
                self.log_message(f"Centering above QR using {algo} algorithm...")
                if algo == "Region Based":
                    qrs = self.mission.center_above_qr_region_based(cap)
                elif algo == "PID Based":
                    qrs = self.mission.center_above_qr_pid_based(cap)

                self.mission.align_with_qr(qrs)
                

                self.mission.land_on_qr()
                
                time.sleep(7)
                self.log_message("Servos actuated")
                self.drone_controller.set_servo(6, 2100)
                time.sleep(7)
                self.drone_controller.arm_and_takeoff(target_alt, cap=cap)
                self.vehicle.home_location = home
                self.drone_controller.set_servo(6, 950)
                # Return to home
                self.log_message("Returning to original home location...")
                self.vehicle.mode = VehicleMode("RTL")
                time.sleep(2)

                self.vehicle.close()
                
                with self.mission_lock:
                    self.mission_running = False
                self.start_mission_btn.config(state="normal")
                self.log_message("Mission completed!")
                
            except Exception as e:
                self.log_message(f"Mission error: {str(e)}")
                with self.mission_lock:
                    self.mission_running = False
                self.start_mission_btn.config(state="normal")
            # finally:
            #     if cap is not None:
            #         cap.release()
        
        threading.Thread(target=mission_thread, daemon=True).start()
    def emergency_rtl(self):
        """Emergency RTL - return to launch immediately"""
        if self.vehicle and self.is_connected:
            self.log_message("! EMERGENCY RTL ACTIVATED!")
            self.vehicle.mode = VehicleMode("RTL")
            with self.mission_lock:
                self.mission_running = False
    
    def run(self):
        """Start the GUI"""
        self.root.mainloop()
        if self.vehicle:
            try:
                self.vehicle.close()
            except:
                pass

def main():
    app = DroneControlGUI()
    app.run()

if __name__ == "__main__":
    main()
