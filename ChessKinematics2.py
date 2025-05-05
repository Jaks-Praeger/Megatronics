import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import os
import math
from IPython.display import HTML
import time
import serial
import json  # Add this import at the top of the file
import re  # Add this import for regex matching

class ChessRobot:
    def __init__(self, base_height=2.8448,
                 link1_length=37.2872,
                 link2_length=36.83, 
                 gripper_height=16.51,
                 chess_square_size=4.07, #4.07
                 board_height=2.5273, #2.5273
                 board_origin_x=-19.1, #-19.1
                 board_origin_y=26.035, #31.625
                 linkage_angle_deg=99.8,
                 linkage_length=4.9784,
                 baseDistFromOrigin=1.453642,
                 gripperDistFromOrigin=3.601466 + 0.70612,
                 home_position=(90, 130, 50)):
        
        """
        Initialize the chess robot with specified dimensions and parameters.
        
        Args:
            base_height: Height of the robot base in cm
            link1_length: Length of the first arm segment in cm
            link2_length: Length of the second arm segment in cm
            gripper_height: Height of the end-effector gripper in cm
            chess_square_size: Size of each chess square in cm
            board_height: Height of the chess board in cm
            board_origin_x: X-coordinate of the chess board origin (a1 square) in cm
            board_origin_y: Y-coordinate of the chess board origin (a1 square) in cm
            linkage_angle_deg: Angle of fixed linkage relative to base arm (degrees)
            linkage_length: Length of fixed linkage in cm
            home_position: Default resting position as (base_angle, shoulder_angle, elbow_angle) in degrees
                           Note: elbow_angle is now the interior angle between links
        """
        # Physical dimensions
        self.base_height = base_height
        self.link1_length = link1_length
        self.link2_length = link2_length
        self.gripper_height = gripper_height
        self.linkage_angle_deg = - 180 + linkage_angle_deg
        self.linkage_length = linkage_length
        self.baseDistFromOrigin = baseDistFromOrigin
        self.gripperDistFromOrigin = gripperDistFromOrigin
        
        # Chess board parameters
        self.chess_square_size = chess_square_size
        self.board_height = board_height
        self.board_origin_x = board_origin_x
        self.board_origin_y = board_origin_y
        
        # Current joint angles in degrees (base rotation, shoulder, elbow)
        self.current_angles = list(home_position)
        self.home_position = home_position
        
        # Gripper state (True = closed, False = open)
        self.gripper_closed = False
        
        # Movement parameters
        self.move_step_size = 3  # degrees per step
        self.z_clearance = 8.0 + self.gripper_height  # cm above pieces for grabbing
        self.z_clearance_moving = self.z_clearance + 15.0  # cm above pieces for moving
        self.tall_piece_clearance = 1.5

        self.sendActualCommands = True

        if self.sendActualCommands:
            self.ser = self.setup_serial()

        self.allowAngles = False
        if not self.sendActualCommands:
            self.allowAngles = True

    def setup_serial(self, port='COM7', baud_rate=115200):
        """Set up serial connection to Arduino"""
        ser = serial.Serial(port, baud_rate, timeout=1)
        ser.setDTR(False)  # Prevents reset
        time.sleep(2)  # Allow connection to establish
        return ser
    
    def safe_serial_write(self, data_string):
        """Safely write to the serial port, with auto-reconnect on failure."""
        if not self.sendActualCommands:
            # print(f"Simulated serial write: {data_string.strip()}")
            return

        for attempt in range(2):  # Try at most twice
            try:
                if not self.ser or not self.ser.is_open:
                    print("Serial port closed. Reconnecting...")
                    self.ser = self.setup_serial()

                self.ser.write(data_string.encode())
                return  # Success, return early

            except serial.SerialException as e:
                print(f"[Attempt {attempt+1}] Serial write failed: {e}")
                try:
                    if self.ser:
                        self.ser.close()
                except:
                    pass  # Ignore if already closed
                self.ser = None  # Reset for retry

        print("⚠️ Serial write ultimately failed. Giving up.")

    def safe_serial_readline(self):
        """Safely read a line from the serial port."""
        if not self.sendActualCommands:
            return None

        try:
            if self.ser and self.safe_serial_in_waiting() > 0:
                return self.ser.readline().decode('utf-8').rstrip()
        except serial.SerialException as e:
            print(f"Serial read failed: {e}")
            for attempt in range(2):  # Try at most twice
                try:
                    if not self.ser or not self.ser.is_open:
                        print("Serial port closed. Reconnecting...")
                        self.ser = self.setup_serial()

                    self.ser.readline().decode('utf-8').rstrip()
                    return  # Success, return early

                except serial.SerialException as e:
                    print(f"[Attempt {attempt+1}] Serial write failed: {e}")
                    try:
                        if self.ser:
                            self.ser.close()
                    except:
                        pass  # Ignore if already closed
                    self.ser = None  # Reset for retry
        return None
    
    def safe_serial_in_waiting(self):
        """Safely get in_waiting from serial without raising ClearCommError."""
        
        if not self.sendActualCommands or not self.ser:
            return 0
        try:
            return self.ser.in_waiting
        except serial.SerialException as e:
            print(f"[in_waiting error] {e}")
            
            try:
                self.ser.close()
            except:
                pass
            self.ser = None
            return 0

    def sendAngleArray(self, angles):
        while not self.allowAngles:
            # print(self.ser)
            # print(self.safe_serial_in_waiting())
            while self.safe_serial_in_waiting() > 0:
                data = self.safe_serial_readline()
                if data == "Calibration complete":
                    self.allowAngles = 1
                print(data)

        data_string = f"{round(angles[0], 2), round(angles[1], 2), round(angles[2], 2), round(self.gripper_closed, 2)}\n"
        print(f"Printing from RPi: {data_string}")

        self.safe_serial_write(data_string)

        if self.sendActualCommands:
            doneWithMove = False
        else:
            doneWithMove = True

        while not doneWithMove:
            data = self.safe_serial_readline()
            if data:
                if data == "Done with move":
                    doneWithMove = True
                # print(data)
        
        time.sleep(0.5)  # Allow time for the command to be processed

            # if self.sendActualCommands:
            #     self.ser.close()

    def algebraic_to_angles(self, chess_position):
        """
        Convert chess algebraic notation (e.g., 'e4') to coordinates by looking up in positions.json.
        
        Args:
            chess_position: Chess position in algebraic notation (e.g., 'e4')
            
        Returns:
            tuple: angles
        """
        # Load the positions from the JSON file
        positions_file = r"c:\Users\dafe2002\Mega\Megatronics\positions.json"
        with open(positions_file, 'r') as file:
            positions = json.load(file)
        
        # Check if the position exists in the JSON file
        if chess_position not in positions:
            raise ValueError(f"Position {chess_position} not found in positions.json")
        
        # Return the coordinates as a tuple
        return tuple(positions[chess_position])

    def algebraic_to_coordinate(self, chess_position):
        """
        Convert chess algebraic notation (e.g., 'e4') to board coordinates.
        
        Args:
            chess_position: Chess position in algebraic notation (e.g., 'e4')
            
        Returns:
            tuple: (x, y) coordinates in cm
        """
        # Check if the input is valid
        if not (len(chess_position) == 2 and 
                'a' <= chess_position[0].lower() <= 'h' and 
                '1' <= chess_position[1] <= '8'):
            raise ValueError(f"Invalid chess position: {chess_position}")
        
        # Convert file (column) from letter to number (a=0, b=1, ..., h=7)
        file_idx = ord(chess_position[0].lower()) - ord('a')
        # Convert rank (row) from character to integer and subtract 1 (1=0, 2=1, ..., 8=7)
        rank_idx = int(chess_position[1]) - 1
        
        # Calculate the center coordinates of the square
        x = self.board_origin_x + (file_idx + 0.5) * self.chess_square_size
        y = self.board_origin_y + (rank_idx + 0.5) * self.chess_square_size
        
        return (x, y)

    def inverse_kinematics(self, x, y, z):

        """
        Calculate joint angles required to reach a given position.

        Args:
            x, y, z: Target position coordinates in cm

        Returns:
            tuple: (base_angle, shoulder_angle, elbow_angle) in degrees
        """

        i = 0
        iterations = 5
        actual_x = x
        actual_y = y
        # print(f"Initial target position: {x}, {y}, {z}")
        base_angle = math.degrees(math.atan2(y, x))
        # print(f"Starting base angle: {base_angle}")

        for i in range(iterations):
            # Calculate the base rotation angle
            base_angle = math.degrees(math.atan2(y, x))
            # swapped sin and cos below because the shift is perpendicular to the base angle
            base_x_shift = self.baseDistFromOrigin * math.sin(math.radians(base_angle))
            base_y_shift = -self.baseDistFromOrigin * math.cos(math.radians(base_angle))
            # this one is also perpendicular, but to the arm and in the opposite direction
            gripper_x_shift = -self.gripperDistFromOrigin * math.sin(math.radians(base_angle))
            gripper_y_shift = self.gripperDistFromOrigin * math.cos(math.radians(base_angle))

            total_x_shift = base_x_shift + gripper_x_shift
            total_y_shift = base_y_shift + gripper_y_shift
            # print(f"Base angle: {base_angle}, X shift: {total_x_shift}, Y shift: {total_y_shift}")

            # Adjust the target position based on the shifts
            x = actual_x - total_x_shift
            y = actual_y - total_y_shift

        # print(f"Adjusted target position: {x}, {y}, {z}")
        # print(f"Base angle after shifts: {base_angle}")

        # Now that we have x and y, we can calculate the shoulder and elbow angles
        r_xy = math.sqrt(x**2 + y**2)
        height_from_shoulder = z - self.base_height
        reach = math.sqrt(r_xy**2 + height_from_shoulder**2)

        # Check if the target is reachable
        max_reach = self.link1_length + self.link2_length
        #if reach > max_reach:
        #    raise ValueError(f"Target position ({x}, {y}, {z}) is out of reach. Maximum reach is {max_reach} cm.")

        i = 0
        linkage_angle_rad = math.radians(self.linkage_angle_deg)
        actual_r_xy = r_xy
        actual_height_from_shoulder = height_from_shoulder

        alpha = math.degrees(math.atan2(height_from_shoulder, r_xy))
        beta = math.degrees(math.acos((self.link1_length**2 + reach**2 - self.link2_length**2) / (2 * self.link1_length * reach)))
        pre_shoulder_angle = alpha + beta
        # print(f"Pre-shoulder angle: {pre_shoulder_angle}")


        for i in range(iterations):
            # Calculate the shoulder angle
            alpha = math.degrees(math.atan2(height_from_shoulder, r_xy))
            beta = math.degrees(math.acos((self.link1_length**2 + reach**2 - self.link2_length**2) / (2 * self.link1_length * reach)))
            pre_shoulder_angle = alpha + beta
            # print(f"Pre-shoulder angle: {pre_shoulder_angle}")

            # with that, subtract the linkage contribution to the target position
            pre_shoulder_angle_rad = math.radians(pre_shoulder_angle)
            linkage_x = self.linkage_length * math.cos(linkage_angle_rad + pre_shoulder_angle_rad)
            linkage_y = self.linkage_length * math.sin(linkage_angle_rad + pre_shoulder_angle_rad)
            # print(f"Linkage contribution: {linkage_x}, {linkage_y}")

            r_xy = actual_r_xy - linkage_x
            height_from_shoulder = actual_height_from_shoulder - linkage_y
            reach = math.sqrt(r_xy**2 + height_from_shoulder**2)

        # Now recalculate the angles with the new target position
        alpha = math.degrees(math.atan2(height_from_shoulder, r_xy))
        beta = math.degrees(math.acos((self.link1_length**2 + reach**2 - self.link2_length**2) / (2 * self.link1_length * reach)))
        shoulder_angle = alpha + beta
        # print(f"Actual shoulder angle: {shoulder_angle}")

        # Use the law of cosines to calculate the elbow angle
        cos_elbow = (self.link1_length**2 + self.link2_length**2 - reach**2) / (2 * self.link1_length * self.link2_length)
        # Clamp to valid range to avoid numerical errors
        cos_elbow = max(min(cos_elbow, 1.0), -1.0)  

        # FIXED: Return the actual interior angle at the elbow
        elbow_angle = math.degrees(math.acos(cos_elbow))

        return (base_angle, shoulder_angle, elbow_angle)

    def forward_kinematics_improved(self, base_angle, shoulder_angle, elbow_angle):
        """
        Calculate the end-effector position based on joint angles.

        Args:
            base_angle: Angle of the base joint in degrees
            shoulder_angle: Angle of the shoulder joint in degrees
            elbow_angle: Angle of the elbow joint in degrees

        Returns:
            tuple: (x, y, z) position coordinates in cm
        """
        # Convert angles to radians
        base_rad = math.radians(base_angle)
        shoulder_rad = math.radians(shoulder_angle)
        elbow_rad = math.radians(elbow_angle)
        
        # Calculate the position in the arm plane (ignoring base rotation for now)
        # First, calculate position at the end of link1
        link1_x = self.link1_length * math.cos(shoulder_rad)
        link1_y = self.link1_length * math.sin(shoulder_rad)
        
        # Calculate the absolute angle of link2 from horizontal
        link2_angle = shoulder_rad - math.radians(180 - elbow_angle)
        
        # Calculate the end position of link2
        link2_x = self.link2_length * math.cos(link2_angle)
        link2_y = self.link2_length * math.sin(link2_angle)
        
        # Calculate arm endpoint in arm plane (before accounting for base rotation)
        arm_plane_x = link1_x + link2_x
        arm_plane_y = link1_y + link2_y
        
        # Account for linkage effect if present
        if hasattr(self, 'linkage_length') and hasattr(self, 'linkage_angle_deg'):
            linkage_angle_rad = math.radians(self.linkage_angle_deg)
            linkage_contribution_x = self.linkage_length * math.cos(linkage_angle_rad + shoulder_rad)
            linkage_contribution_y = self.linkage_length * math.sin(linkage_angle_rad + shoulder_rad)
            arm_plane_x += linkage_contribution_x
            arm_plane_y += linkage_contribution_y
        
        # Apply base rotation to get the final x, y coordinates
        x_rotated = arm_plane_x * math.cos(base_rad)
        y_rotated = arm_plane_x * math.sin(base_rad)
        
        # Apply base and gripper offsets
        if hasattr(self, 'baseDistFromOrigin'):
            x_rotated += self.baseDistFromOrigin * math.sin(base_rad)
            y_rotated += -self.baseDistFromOrigin * math.cos(base_rad)
        
        if hasattr(self, 'gripperDistFromOrigin'):
            x_rotated += -self.gripperDistFromOrigin * math.sin(base_rad)
            y_rotated += self.gripperDistFromOrigin * math.cos(base_rad)
        
        # Calculate final z coordinate
        z = self.base_height + arm_plane_y
        
        return (x_rotated, y_rotated, z)

    def move_joints(self, target_angles, simulate=False):
        """
        Move the robot joints to the specified angles using small steps.
        
        Args:
            target_angles: Target (base_angle, shoulder_angle, elbow_angle) in degrees
            simulate: If True, don't execute actual movement commands
            
        Returns:
            None
        """
        # # Calculate the number of steps for the largest angle change
        # angle_changes = [abs(target - current) for target, current in zip(target_angles, self.current_angles)]
        # max_change = max(angle_changes)
        # num_steps = max(1, int(max_change / self.move_step_size))
        
        # # Generate intermediate angles for smooth movement
        # for step in range(1, num_steps + 1):
        #     # Calculate interpolated angles for this step
        #     intermediate_angles = [
        #         current + (target - current) * step / num_steps
        #         for current, target in zip(self.current_angles, target_angles)
        #     ]
            
        #     if not simulate:
        #         # Here you would send the actual commands to your robot hardware
        #         # print(f"Moving to angles: Base={intermediate_angles[0]:.1f}°, "
        #               # f"Shoulder={intermediate_angles[1]:.1f}°, Elbow={intermediate_angles[2]:.1f}°")
        #         self.send_to_motors(intermediate_angles)
                
        #         time.sleep(0.05)  # Simulate movement time
            
        #     # Append a frame to the visualization if visualizer is available
        #     if hasattr(self, 'visualizer'):
        #         self.visualizer.append_frame(intermediate_angles, self.gripper_closed)


        # Update current angles
        # self.current_angles = intermediate_angles
            
        # Ensure we arrive exactly at the target angles
        if not simulate:
            # print(f"Final position: Base={target_angles[0]:.1f}°, "
                  # f"Shoulder={target_angles[1]:.1f}°, Elbow={target_angles[2]:.1f}°")
            self.send_to_motors(target_angles)

        # Append a final frame to the visualization
        if hasattr(self, 'visualizer'):
            self.visualizer.append_frame(target_angles, self.gripper_closed)
            
        self.current_angles = list(target_angles)

    def send_to_motors(self, angles):
        """
        Send the joint angles to the motor controllers.
        
        Args:
            angles: (base_angle, shoulder_angle, elbow_angle) in degrees
            
        Returns:
            None
        """
        
        self.sendAngleArray(angles)

    def actuate_gripper(self, close):
        """
        Open or close the gripper.
        
        Args:
            close: True to close the gripper, False to open it
            
        Returns:
            None
        """
        if close != self.gripper_closed:
            print(f"{'Closing' if close else 'Opening'} gripper")
            # Here you would send the actual gripper command to your hardware
            # self.send_gripper_command(close)
            time.sleep(0.2)  # Allow time for gripper to actuate
            self.gripper_closed = close

        # send command
        time.sleep(0.2)
        self.send_to_motors(self.current_angles)
        time.sleep(0.2)


    def move_to_position(self, x, y, z, avoid_collisions=True):
        """
        Move the end effector to a specific position.
        
        Args:
            x, y, z: Target position coordinates in cm
            avoid_collisions: If True, move up before changing XY position
            
        Returns:
            None
        """
        current_x, current_y, current_z = self.forward_kinematics_improved(*self.current_angles)
        
        if avoid_collisions:
            # First move up to clearance height if needed
            if current_z < self.board_height + self.z_clearance_moving:
                clearance_angles = self.inverse_kinematics(current_x, current_y, 
                                                          self.board_height + self.z_clearance_moving)
                self.move_joints(clearance_angles)
            
            # Then move in XY plane at safe height
            safe_height_angles = self.inverse_kinematics(x, y, self.board_height + self.z_clearance_moving)
            self.move_joints(safe_height_angles)
        
        # Finally move to the exact target position
        target_angles = self.inverse_kinematics(x, y, z)
        self.move_joints(target_angles)

    def return_to_home(self):
        """Move the robot back to its home position."""
        print("Returning to home position")
        self.move_joints(self.home_position)
        self.actuate_gripper(False)  # Open gripper

    def getSquarePositionAngles(self,pos):
        x, y = self.algebraic_to_coordinate(pos)
        return self.inverse_kinematics(x, y, self.board_height + self.z_clearance)

    def move_piece(self, from_pos, to_pos, tall_piece=False, use_calibrated_coords=True):
        """Helper function to move a piece and update visualization"""
        # Convert chess positions to coordinates
        if not use_calibrated_coords:
            from_x, from_y = self.algebraic_to_coordinate(from_pos)
            to_x, to_y = self.algebraic_to_coordinate(to_pos)
            grip_z = self.board_height + self.z_clearance
            place_z = self.board_height + self.z_clearance
        else:
            # Try to load calibrated coordinates from JSON file
            current_dir = os.path.dirname(os.path.abspath(__file__))
            coords_file = os.path.join(current_dir, "chessCoords.json")
            
            try:
                with open(coords_file, 'r') as file:
                    coords_dict = json.load(file)
                    
                # Check if both positions have calibrated coordinates
                if from_pos in coords_dict and to_pos in coords_dict:
                    from_x, from_y, grip_z = coords_dict[from_pos]
                    to_x, to_y, place_z = coords_dict[to_pos]
                    print(f"Using calibrated coordinates for {from_pos} and {to_pos}")
                else:
                    # Fall back to calculated coordinates if calibrated ones aren't available
                    missing_pos = []
                    if from_pos not in coords_dict:
                        missing_pos.append(from_pos)
                    if to_pos not in coords_dict:
                        missing_pos.append(to_pos)
                    print(f"Calibrated coordinates not found for {', '.join(missing_pos)}. Using calculated values.")
                    
                    # Get calculated coordinates for positions without calibration
                    if from_pos in coords_dict:
                        from_x, from_y, grip_z = coords_dict[from_pos]
                    else:
                        from_x, from_y = self.algebraic_to_coordinate(from_pos)
                        grip_z = self.board_height + self.z_clearance
                        
                    if to_pos in coords_dict:
                        to_x, to_y, place_z = coords_dict[to_pos]
                    else:
                        to_x, to_y = self.algebraic_to_coordinate(to_pos)
                        place_z = self.board_height + self.z_clearance
                        
            except (FileNotFoundError, json.JSONDecodeError) as e:
                print(f"Error loading calibrated coordinates: {e}")
                print("Using calculated coordinates instead.")
                from_x, from_y = self.algebraic_to_coordinate(from_pos)
                to_x, to_y = self.algebraic_to_coordinate(to_pos)
                grip_z = self.board_height + self.z_clearance
                place_z = self.board_height + self.z_clearance
                
        # Calculate Z heights for movements
        piece_top_z = self.board_height + self.z_clearance_moving
        hover_z = self.board_height + self.z_clearance_moving

        # Adjust for tall pieces
        if tall_piece:
            grip_z += self.tall_piece_clearance
            place_z += self.tall_piece_clearance

        grip_z -= self.tall_piece_clearance
        place_z -= self.tall_piece_clearance
        
        
        # 1. Open gripper
        self.actuate_gripper(True)
        
        # 2. Move above the piece to pick
        self.move_to_position(from_x, from_y-2, hover_z)
        
        # 3. Lower to grip the piece
        self.move_to_position(from_x, from_y, grip_z, avoid_collisions=False)
        
        # 4. Close gripper to grab the piece
        self.actuate_gripper(False)
        
        # 5. Lift the piece up
        self.move_to_position(from_x, from_y, piece_top_z, avoid_collisions=False)
        
        # 6. Move to destination square at safe height
        self.move_to_position(to_x, to_y, piece_top_z)
        
        # 7. Lower piece to the board
        self.move_to_position(to_x, to_y, place_z, avoid_collisions=False)
        
        # 8. Release the piece
        self.actuate_gripper(True)
        
        # 9. Move up away from the piece
        self.move_to_position(to_x, to_y, hover_z, avoid_collisions=False)
        
        # 10. Return to home position
        self.move_joints(self.home_position)

        time.sleep(2)

    def capture_piece(self, pos, tall_piece=False):

        self.move_piece(pos, 'capture', tall_piece)
        # """Helper function to capture a piece and update visualization"""
        # # Convert chess position to coordinates
        # to_x, to_y = self.algebraic_to_coordinate(pos)
        # store_x, store_y = captured_storage
        
        # # 1. Open gripper
        # self.actuate_gripper(False)
        
        # # 2. Move to and pick up the captured piece
        # self.move_to_position(to_x, to_y, self.board_height + self.chess_square_size * 0.3)
        # self.actuate_gripper(True)
        
        # # 3. Update visualization piece state if enabled
        # if hasattr(self, 'visualizer'):
        #     self.visualizer.update_piece_position(pos, None, is_capture=True)
        
        # # 4. Lift the captured piece
        # self.move_to_position(to_x, to_y, self.board_height + self.chess_square_size * 0.7, avoid_collisions=False)
        
        # # 5. Move the captured piece to storage area
        # self.move_to_position(store_x, store_y, self.board_height + self.chess_square_size * 0.7)
        
        # # 6. Place the captured piece
        # self.move_to_position(store_x, store_y, self.board_height + 0.1, avoid_collisions=False)
        # self.actuate_gripper(False)
        
        # # 7. Lift up from storage area
        # self.move_to_position(store_x, store_y, self.board_height + self.chess_square_size * 0.7, avoid_collisions=False)

    def calibrateCoordinates(self):
        """
        Interactive calibration function to fine-tune the position of each chess square.
        Allows manual adjustment of x, y, z coordinates for each position.
        Supports dynamic step sizes (e.g., "x+5" for 5cm adjustment).
        Saves coordinates to chessCoords.json.
        """
        
        # Define all chess positions
        files = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
        ranks = ['1', '2', '3', '4', '5', '6', '7', '8']
        positions = [f+r for f in files for r in ranks]
        
        # Initialize coordinates dictionary
        # Get the directory of the current Python file
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # Construct the path to the JSON file in the same directory
        coords_file = os.path.join(current_dir, "chessCoords.json")
        
        # Load existing coordinates if file exists
        if os.path.exists(coords_file):
            try:
                with open(coords_file, 'r') as file:
                    coords_dict = json.load(file)
                print(f"Loaded existing coordinates from {coords_file}")
            except:
                coords_dict = {}
                print(f"Could not load {coords_file}, starting fresh")
        else:
            coords_dict = {}
            print(f"Creating new coordinates file at {coords_file}")
        
        print("\n===== CHESS BOARD CALIBRATION =====")
        print("For each position, the arm will move to the current saved/calculated position.")
        print("You can then adjust x, y, z coordinates until the position is correct.")
        print("Commands:")
        print("  x+/x- [value]: Adjust x coordinate (e.g., 'x+0.5' or 'x-2')")
        print("  y+/y- [value]: Adjust y coordinate (e.g., 'y+1' or 'y-0.2')")
        print("  z+/z- [value]: Adjust z coordinate (e.g., 'z+0.1' or 'z-3')")
        print("  done: Save position and move to next square")
        print("  skip: Skip to next position without saving")
        print("  quit: Save and exit calibration")
        print("==============================\n")
        
        # Regex pattern to match adjustment commands like "x+0.5" or "z-2"
        adjustment_pattern = re.compile(r'^([xyz])([+-])(\d*\.?\d*)$')
        
        try:
            for position in positions:
                # Skip positions that are already calibrated if user wants
                if position in coords_dict:
                    choice = input(f"Position {position} already calibrated. Recalibrate? (y/n): ")
                    if choice.lower() != 'y':
                        continue
                
                print(f"\nMoving to chess position: {position}")
                
                # Get initial coordinates from algebraic_to_coordinate
                initial_x, initial_y = self.algebraic_to_coordinate(position)
                initial_z = self.board_height + self.z_clearance
                
                # Use existing calibrated coordinates if available
                if position in coords_dict:
                    x, y, z = coords_dict[position]
                    print(f"Using existing calibrated coordinates: x={x}, y={y}, z={z}")
                else:
                    x, y, z = initial_x, initial_y, initial_z
                    print(f"Starting with calculated coordinates: x={x}, y={y}, z={z}")
                
                # Move to current position
                try:
                    print(f"\n z is : {z}")
                    self.move_to_position(x, y, z)
                except ValueError as e:
                    print(f"Error moving to position: {e}")
                    print("Using default position instead")
                    self.move_joints(self.home_position)
                    x, y, z = initial_x, initial_y, initial_z
                    self.move_to_position(x, y, z)
                
                # Default adjustment step in cm (used when no value is specified)
                default_step = 0.5
                
                # Adjustment loop
                while True:
                    command = input(f"Position {position} (x={x:.2f}, y={y:.2f}, z={z:.2f}) - Enter command: ")
                    
                    if command == "done":
                        coords_dict[position] = [x, y, z]
                        print(f"Saved position {position}: {[x, y, z]}")
                        break
                        
                    elif command == "skip":
                        print(f"Skipping position {position}")
                        break
                        
                    elif command == "quit":
                        print("Saving coordinates and exiting calibration...")
                        with open(coords_file, 'w') as file:
                            json.dump(coords_dict, file, indent=2)
                        print(f"Coordinates saved to {coords_file}")
                        self.return_to_home()
                        return
                    
                    # Parse adjustment commands (e.g., "x+0.5", "y-2", "z+")
                    match = adjustment_pattern.match(command)
                    if match:
                        axis = match.group(1)  # x, y, or z
                        direction = match.group(2)  # + or -
                        
                        # If no value specified, use default step
                        value_str = match.group(3)
                        step = float(value_str) if value_str else default_step
                        
                        # Apply the adjustment
                        if axis == 'x':
                            x += step if direction == '+' else -step
                        elif axis == 'y':
                            y += step if direction == '+' else -step
                        elif axis == 'z':
                            z += step if direction == '+' else -step
                        
                        # Move to the new position
                        print(f"Adjusting {axis} by {direction}{step}")
                        self.move_to_position(x, y, z, avoid_collisions=False)
                    
                    # Handle traditional x+, y-, etc. commands for backward compatibility
                    elif command in ["x+", "x-", "y+", "y-", "z+", "z-"]:
                        axis = command[0]
                        direction = command[1]
                        
                        if axis == 'x':
                            x += default_step if direction == '+' else -default_step
                        elif axis == 'y':
                            y += default_step if direction == '+' else -default_step
                        elif axis == 'z':
                            z += default_step if direction == '+' else -default_step
                        
                        print(f"Adjusting {axis} by {direction}{default_step}")
                        self.move_to_position(x, y, z, avoid_collisions=False)
                        
                    elif command.startswith("step"):
                        try:
                            default_step = float(command.split()[1])
                            print(f"Default adjustment step changed to {default_step} cm")
                        except:
                            print(f"Invalid step value. Using {default_step} cm")
                        
                    else:
                        print("Unknown command. Available commands:")
                        print("  x+/x- [value]: Adjust x coordinate (e.g., 'x+0.5' or 'x-2')")
                        print("  y+/y- [value]: Adjust y coordinate (e.g., 'y+1' or 'y-0.2')")
                        print("  z+/z- [value]: Adjust z coordinate (e.g., 'z+0.1' or 'z-3')")
                        print(f"  step <value>: Change default adjustment step (current: {default_step})")
                        print("  done: Save position and move to next square")
                        print("  skip: Skip to next position without saving")
                        print("  quit: Save and exit calibration")
            
            # Save final coordinates
            with open(coords_file, 'w') as file:
                json.dump(coords_dict, file, indent=2)
            print(f"Calibration complete! Coordinates saved to {coords_file}")
            self.return_to_home()
            
        except KeyboardInterrupt:
            print("\nCalibration interrupted!")
            choice = input("Save current progress? (y/n): ")
            if choice.lower() == 'y':
                with open(coords_file, 'w') as file:
                    json.dump(coords_dict, file, indent=2)
                print(f"Coordinates saved to {coords_file}")
            self.return_to_home()

def run_chess_robot_demo():
    # Create robot with visualization-friendly parameters
    # Note: Modified link lengths to create a better visualization
    robot = ChessRobot()
    
    # Set up improved visualization
    # setup_improved_visualization(robot)
    
    # 1. King's pawn opening (e2 to e4)
    # print("\nMoving pawn from e2 to e4")
    #robot.move_piece('f3', 'e5')
    
    # 2. Knight to f3
    # print("\nMoving knight from g1 to f3")
    # robot.move_piece('a1', 'a2')
    # robot.calibrateCoordinates()
    # robot.capture_piece('f3')
    
    while 1:
        move_input = input("enter in old and new position e.g 'a1 b3': ")

        if move_input == 'done':
            break

        start_pos, end_pos = move_input.split()

        robot.move_piece(start_pos, end_pos, 1)
    
    # 3. Queen's pawn opening (d2 to d4)
    # print("\nMoving pawn from d2 to d4")
    # robot.move_piece('d2', 'd4')
    
    # 4. Knight captures on e4
    # print("\nCapturing with knight from f3 to e4")
    
    # Save the animation
    # print("\nSaving animation...")
    # if hasattr(robot, 'visualizer'):
    #     robot.visualizer.create_animation(filename="chess_robot_demo.gif", method="imageio", fps=10)

    # print(robot.algebraic_to_angles('g6'))

    print("\nDemo completed!")
    if robot.sendActualCommands:
        if robot.ser.is_open:
            robot.ser.close()

if __name__ == "__main__":
    run_chess_robot_demo()