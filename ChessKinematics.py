import math
import numpy as np
import time
import serial

def setup_serial(port='/dev/ttyACM0', baud_rate=9600):
    """Set up serial connection to Arduino"""
    ser = serial.Serial(port, baud_rate, timeout=1)
    time.sleep(2)  # Allow connection to establish
    return ser

def sendAngleArray(ser, angles):
    """
    Send an array of 3 angles to Arduino
    
    Args:
        ser: Serial connection object
        angles: List of 3 angle values
    """
    # Format the three angles with comma separators
    data_string = f"{angles[0]},{angles[1]},{angles[2]}\n"
    
    # Send the data
    ser.write(data_string.encode())

class ChessRobot:
    def __init__(self, base_height=10.0, link1_length=60.0, link2_length=60.0, 
                 gripper_height=2.0, chess_square_size=5.0, board_height=1.0,
                 board_origin_x=0.0, board_origin_y=10.0, home_position=(0, 90, 0)):
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
            home_position: Default resting position as (base_angle, shoulder_angle, elbow_angle) in degrees
        """
        # Physical dimensions
        self.base_height = base_height
        self.link1_length = link1_length
        self.link2_length = link2_length
        self.gripper_height = gripper_height
        
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
        self.move_step_size = 5  # degrees per step
        self.z_clearance = 10.0  # cm above pieces for safe movement

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
        # Calculate the distance from the base to the target point in the XY plane
        r_xy = math.sqrt(x**2 + y**2)
        
        # Calculate the height relative to the shoulder joint
        height_from_shoulder = z - self.base_height
        
        # Calculate the reach distance (from shoulder to target)
        reach = math.sqrt(r_xy**2 + height_from_shoulder**2)
        
        # Check if the target is reachable
        max_reach = self.link1_length + self.link2_length
        if reach > max_reach:
            raise ValueError(f"Target position ({x}, {y}, {z}) is out of reach. Maximum reach is {max_reach} cm.")
        
        # Calculate the base rotation angle
        base_angle = math.degrees(math.atan2(y, x))
        
        # Use the law of cosines to calculate the elbow angle
        cos_elbow = (self.link1_length**2 + self.link2_length**2 - reach**2) / (2 * self.link1_length * self.link2_length)
        # Clamp to valid range to avoid numerical errors
        cos_elbow = max(min(cos_elbow, 1.0), -1.0)  
        elbow_angle = math.degrees(math.acos(cos_elbow))
        
        # Calculate the shoulder angle
        alpha = math.degrees(math.atan2(height_from_shoulder, r_xy))
        beta = math.degrees(math.acos((self.link1_length**2 + reach**2 - self.link2_length**2) / (2 * self.link1_length * reach)))
        shoulder_angle = alpha + beta
        
        return (base_angle, shoulder_angle, 180 - elbow_angle)  # Elbow angle is measured from fully extended

    def forward_kinematics(self, base_angle, shoulder_angle, elbow_angle):
        """
        Calculate the end-effector position given the joint angles.
        
        Args:
            base_angle: Base rotation angle in degrees
            shoulder_angle: Shoulder angle in degrees
            elbow_angle: Elbow angle in degrees
            
        Returns:
            tuple: (x, y, z) coordinates in cm
        """
        # Convert angles to radians
        base_rad = math.radians(base_angle)
        shoulder_rad = math.radians(shoulder_angle)
        elbow_rad = math.radians(180 - elbow_angle)  # Convert from our convention
        
        # Calculate the position of the elbow joint
        elbow_x = self.link1_length * math.cos(shoulder_rad) * math.cos(base_rad)
        elbow_y = self.link1_length * math.cos(shoulder_rad) * math.sin(base_rad)
        elbow_z = self.base_height + self.link1_length * math.sin(shoulder_rad)
        
        # Calculate the position of the end effector
        x = elbow_x + self.link2_length * math.cos(shoulder_rad + elbow_rad) * math.cos(base_rad)
        y = elbow_y + self.link2_length * math.cos(shoulder_rad + elbow_rad) * math.sin(base_rad)
        z = elbow_z + self.link2_length * math.sin(shoulder_rad + elbow_rad)
        
        return (x, y, z)

    def move_joints(self, target_angles, simulate=False):
        """
        Move the robot joints to the specified angles using small steps.
        
        Args:
            target_angles: Target (base_angle, shoulder_angle, elbow_angle) in degrees
            simulate: If True, don't execute actual movement commands
            
        Returns:
            None
        """
        # Calculate the number of steps for the largest angle change
        angle_changes = [abs(target - current) for target, current in zip(target_angles, self.current_angles)]
        max_change = max(angle_changes)
        num_steps = max(1, int(max_change / self.move_step_size))
        
        # Generate intermediate angles for smooth movement
        for step in range(1, num_steps + 1):
            # Calculate interpolated angles for this step
            intermediate_angles = [
                current + (target - current) * step / num_steps
                for current, target in zip(self.current_angles, target_angles)
            ]
            
            if not simulate:
                # Here you would send the actual commands to your robot hardware
                print(f"Moving to angles: Base={intermediate_angles[0]:.1f}°, "
                      f"Shoulder={intermediate_angles[1]:.1f}°, Elbow={intermediate_angles[2]:.1f}°")
                # Example code for sending to hardware:
                # self.send_to_motors(intermediate_angles)
                
                time.sleep(0.05)  # Simulate movement time
            
            # Append a frame to the visualization if visualizer is available
            if hasattr(self, 'visualizer'):
                self.visualizer.append_frame(intermediate_angles, self.gripper_closed)

            # Update current angles
            self.current_angles = intermediate_angles
            
        # Ensure we arrive exactly at the target angles
        if not simulate:
            print(f"Final position: Base={target_angles[0]:.1f}°, "
                  f"Shoulder={target_angles[1]:.1f}°, Elbow={target_angles[2]:.1f}°")
            # self.send_to_motors(target_angles)

        # Append a final frame to the visualization
        if hasattr(self, 'visualizer'):
            self.visualizer.append_frame(target_angles, self.gripper_closed)
            
        self.current_angles = list(target_angles)

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
            time.sleep(0.5)  # Allow time for gripper to actuate
            self.gripper_closed = close

    def move_to_position(self, x, y, z, avoid_collisions=True):
        """
        Move the end effector to a specific position.
        
        Args:
            x, y, z: Target position coordinates in cm
            avoid_collisions: If True, move up before changing XY position
            
        Returns:
            None
        """
        current_x, current_y, current_z = self.forward_kinematics(*self.current_angles)
        
        if avoid_collisions:
            # First move up to clearance height if needed
            if current_z < self.board_height + self.z_clearance:
                clearance_angles = self.inverse_kinematics(current_x, current_y, 
                                                          self.board_height + self.z_clearance)
                self.move_joints(clearance_angles)
            
            # Then move in XY plane at safe height
            safe_height_angles = self.inverse_kinematics(x, y, self.board_height + self.z_clearance)
            self.move_joints(safe_height_angles)
        
        # Finally move to the exact target position
        target_angles = self.inverse_kinematics(x, y, z)
        self.move_joints(target_angles)

    def pick_and_place_piece(self, from_pos, to_pos):
        """
        Pick up a chess piece from one position and place it at another.
        
        Args:
            from_pos: Starting chess position (e.g., 'e2')
            to_pos: Target chess position (e.g., 'e4')
            
        Returns:
            None
        """
        print(f"\nMoving piece from {from_pos} to {to_pos}")
        
        # Convert chess positions to coordinates
        from_x, from_y = self.algebraic_to_coordinate(from_pos)
        to_x, to_y = self.algebraic_to_coordinate(to_pos)
        
        # Calculate Z heights for different operations
        piece_top_z = self.board_height + self.chess_square_size * 0.7  # Approximate chess piece height
        hover_z = self.board_height + self.chess_square_size * 0.3      # Height to hover above pieces
        grip_z = self.board_height + self.chess_square_size * 0.3       # Height to grip the piece
        place_z = self.board_height + 0.1                               # Height to place the piece
        
        # 1. Open gripper
        self.actuate_gripper(False)
        
        # 2. Move above the piece to pick
        self.move_to_position(from_x, from_y, hover_z)
        
        # 3. Lower to grip the piece
        self.move_to_position(from_x, from_y, grip_z, avoid_collisions=False)
        
        # 4. Close gripper to grab the piece
        self.actuate_gripper(True)
        
        # 5. Lift the piece up
        self.move_to_position(from_x, from_y, piece_top_z, avoid_collisions=False)
        
        # 6. Move to destination square at safe height
        self.move_to_position(to_x, to_y, piece_top_z)
        
        # 7. Lower piece to the board
        self.move_to_position(to_x, to_y, place_z, avoid_collisions=False)
        
        # 8. Release the piece
        self.actuate_gripper(False)
        
        # 9. Move up away from the piece
        self.move_to_position(to_x, to_y, hover_z, avoid_collisions=False)
        
        # 10. Return to home position
        self.return_to_home()
        
        print(f"Successfully moved piece from {from_pos} to {to_pos}")

    def return_to_home(self):
        """Move the robot back to its home position."""
        print("Returning to home position")
        self.move_joints(self.home_position)
        self.actuate_gripper(False)  # Open gripper

    def capture_piece(self, pos, captured_storage=(35, 35)):
        """
        Capture a chess piece by first removing the captured piece.
        
        Args:
            from_pos: Starting chess position (e.g., 'e2')
            to_pos: Target chess position with piece to capture (e.g., 'e4')
            captured_storage: (x, y) coordinates where captured pieces are stored
            
        Returns:
            None
        """
        print(f"\nCapturing piece at {pos}")
        
        # Convert chess position to coordinates
        to_x, to_y = self.algebraic_to_coordinate(pos)
        store_x, store_y = captured_storage
        
        # 1. Open gripper
        self.actuate_gripper(False)
        
        # 2. Move to and pick up the captured piece
        self.move_to_position(to_x, to_y, self.board_height + self.chess_square_size * 0.3)
        self.actuate_gripper(True)
        
        # 3. Lift the captured piece
        self.move_to_position(to_x, to_y, self.board_height + self.chess_square_size * 0.7, avoid_collisions=False)
        
        # 4. Move the captured piece to storage area
        self.move_to_position(store_x, store_y, self.board_height + self.chess_square_size * 0.7)
        
        # 5. Place the captured piece
        self.move_to_position(store_x, store_y, self.board_height + 0.1, avoid_collisions=False)
        self.actuate_gripper(False)
        
        # 6. Lift up from storage area
        self.move_to_position(store_x, store_y, self.board_height + self.chess_square_size * 0.7, avoid_collisions=False)

    def calibrate(self):
        """
        Perform a calibration sequence for the robot.
        This would be expanded with actual calibration logic for a real robot.
        """
        print("Starting calibration sequence...")
        
        # Move to home position
        self.return_to_home()
        
        # Open and close gripper
        self.actuate_gripper(False)
        time.sleep(1)
        self.actuate_gripper(True)
        time.sleep(1)
        self.actuate_gripper(False)
        
        # Move to each corner of the board to verify calibration
        corners = ['a1', 'a8', 'h8', 'h1']
        safe_z = self.board_height + self.z_clearance
        
        for corner in corners:
            print(f"Moving to corner {corner}")
            x, y = self.algebraic_to_coordinate(corner)
            self.move_to_position(x, y, safe_z)
            time.sleep(1)
        
        # Return to home position
        self.return_to_home()
        
        print("Calibration complete")

    def setup_visualization(self):
        """
        Set up the 3D visualization for the robot.
        """
        try:
            import matplotlib.pyplot as plt
            from matplotlib.animation import FuncAnimation
            from mpl_toolkits.mplot3d import Axes3D
            import numpy as np
            
            print("Setting up 3D visualization...")
            
            # Create the visualizer class
            class RobotVisualizer:
                def __init__(self, robot):
                    self.robot = robot
                    self.fig = plt.figure(figsize=(12, 10))
                    self.ax = self.fig.add_subplot(111, projection='3d')
                    self.frames = []
                    self.pieces = {}  # Chess pieces on the board
                    self.colors = {
                        'base': 'gray',
                        'link1': 'blue',
                        'link2': 'green',
                        'gripper': 'red',
                        'chess_light': 'wheat',
                        'chess_dark': 'saddlebrown',
                        'piece_white': 'whitesmoke',
                        'piece_black': 'dimgray'
                    }
                    
                    # Initial setup of the board
                    self.setup_initial_pieces()
                    self.setup_3d_scene()
                    self.append_frame(robot.current_angles, robot.gripper_closed)
                
                def setup_initial_pieces(self):
                    """Set up initial chess pieces for the simulation."""
                    # White pawns on rank 2
                    for file_idx in range(8):
                        file_char = chr(ord('a') + file_idx)
                        self.pieces[f"{file_char}2"] = ('white', False)
                    
                    # Black pawns on rank 7
                    for file_idx in range(8):
                        file_char = chr(ord('a') + file_idx)
                        self.pieces[f"{file_char}7"] = ('black', False)
                    
                    # White pieces on rank 1
                    for file_idx in range(8):
                        file_char = chr(ord('a') + file_idx)
                        self.pieces[f"{file_char}1"] = ('white', False)
                    
                    # Black pieces on rank 8
                    for file_idx in range(8):
                        file_char = chr(ord('a') + file_idx)
                        self.pieces[f"{file_char}8"] = ('black', False)
                
                def setup_3d_scene(self):
                    """Set up the 3D scene with chess board and pieces."""
                    # Clear the axis
                    self.ax.clear()
                    
                    # Draw the chess board
                    self.draw_chessboard()
                    
                    # Draw chess pieces based on their current state
                    for pos, (color, captured) in self.pieces.items():
                        if not captured:
                            self.draw_chess_piece(pos, color, captured)
                    
                    # Set the axis properties
                    self.ax.set_xlabel('X (cm)')
                    self.ax.set_ylabel('Y (cm)')
                    self.ax.set_zlabel('Z (cm)')
                    
                    # Set view limits
                    max_dim = max(self.robot.board_origin_x + 8*self.robot.chess_square_size, 
                                  self.robot.board_origin_y + 8*self.robot.chess_square_size,
                                  self.robot.base_height + self.robot.link1_length + self.robot.link2_length)
                    
                    # Add some margin
                    margin = 10
                    self.ax.set_xlim([-margin, max_dim + margin])
                    self.ax.set_ylim([-margin, max_dim + margin])
                    self.ax.set_zlim([0, max_dim])
                    
                    # Set an isometric-like view
                    self.ax.view_init(elev=30, azim=45)
                    
                    # Set the title
                    self.ax.set_title('Chess Robot 3D Visualization')
                
                def draw_chessboard(self):
                    """Draw the chess board in 3D space."""
                    # Create the chess board grid
                    for i in range(8):  # files a-h
                        for j in range(8):  # ranks 1-8
                            x = self.robot.board_origin_x + i * self.robot.chess_square_size
                            y = self.robot.board_origin_y + j * self.robot.chess_square_size
                            z = self.robot.board_height
                            
                            # Determine square color
                            color = self.colors['chess_light'] if (i + j) % 2 == 0 else self.colors['chess_dark']
                            
                            # Create a square as a rectangular plane
                            xx, yy = np.meshgrid([x, x + self.robot.chess_square_size], 
                                                [y, y + self.robot.chess_square_size])
                            zz = np.ones_like(xx) * z
                            
                            self.ax.plot_surface(xx, yy, zz, color=color, edgecolor='black', linewidth=0.5, alpha=0.8)
                
                def draw_chess_piece(self, position, piece_color='white', captured=False):
                    """Draw a chess piece at the specified position."""
                    if isinstance(position, str):
                        x, y = self.robot.algebraic_to_coordinate(position)
                    else:
                        x, y = position
                        
                    z = self.robot.board_height
                    
                    # Piece height
                    piece_height = self.robot.chess_square_size * 0.8
                    
                    if not captured:
                        # Create a simple cylinder to represent the piece
                        r = self.robot.chess_square_size * 0.3  # Radius
                        theta = np.linspace(0, 2*np.pi, 20)
                        z_vals = np.linspace(z, z + piece_height, 10)
                        theta_grid, z_grid = np.meshgrid(theta, z_vals)
                        
                        x_grid = x + r * np.cos(theta_grid)
                        y_grid = y + r * np.sin(theta_grid)
                        
                        color = self.colors['piece_white'] if piece_color == 'white' else self.colors['piece_black']
                        self.ax.plot_surface(x_grid, y_grid, z_grid, color=color, alpha=0.9)
                
                def get_joint_positions(self, angles):
                    """
                    Calculate positions of all joints for visualization.
                    
                    Args:
                        angles: Joint angles (base_angle, shoulder_angle, elbow_angle) in degrees
                        
                    Returns:
                        tuple: Base, shoulder, elbow, and end effector positions as (x, y, z) tuples
                    """
                    base_angle, shoulder_angle, elbow_angle = angles
                    
                    # Base position
                    base_pos = (0, 0, 0)
                    
                    # Shoulder position
                    shoulder_pos = (0, 0, self.robot.base_height)
                    
                    # Convert angles to radians
                    base_rad = math.radians(base_angle)
                    shoulder_rad = math.radians(shoulder_angle)
                    elbow_rad = math.radians(180 - elbow_angle)
                    
                    # Calculate elbow position
                    elbow_x = self.robot.link1_length * math.cos(shoulder_rad) * math.cos(base_rad)
                    elbow_y = self.robot.link1_length * math.cos(shoulder_rad) * math.sin(base_rad)
                    elbow_z = self.robot.base_height + self.robot.link1_length * math.sin(shoulder_rad)
                    elbow_pos = (elbow_x, elbow_y, elbow_z)
                    
                    # Calculate end effector position
                    ee_x = elbow_x + self.robot.link2_length * math.cos(shoulder_rad + elbow_rad) * math.cos(base_rad)
                    ee_y = elbow_y + self.robot.link2_length * math.cos(shoulder_rad + elbow_rad) * math.sin(base_rad)
                    ee_z = elbow_z + self.robot.link2_length * math.sin(shoulder_rad + elbow_rad)
                    ee_pos = (ee_x, ee_y, ee_z)
                    
                    return base_pos, shoulder_pos, elbow_pos, ee_pos
                
                def draw_robot(self, angles, gripper_closed=False):
                    """
                    Draw the robot at the given joint angles.
                    
                    Args:
                        angles: Joint angles (base_angle, shoulder_angle, elbow_angle) in degrees
                        gripper_closed: Whether the gripper is closed
                    """
                    # Get joint positions
                    base_pos, shoulder_pos, elbow_pos, ee_pos = self.get_joint_positions(angles)
                    
                    # Draw base cylinder
                    base_radius = 3
                    theta = np.linspace(0, 2*np.pi, 20)
                    z = np.linspace(0, self.robot.base_height, 10)
                    theta_grid, z_grid = np.meshgrid(theta, z)
                    x_grid = base_radius * np.cos(theta_grid)
                    y_grid = base_radius * np.sin(theta_grid)
                    
                    # Draw the robot's arm segments
                    # Base to shoulder
                    self.ax.plot([base_pos[0]], [base_pos[1]], [base_pos[2]], 'ko', markersize=10)
                    self.ax.plot([base_pos[0], shoulder_pos[0]], 
                                 [base_pos[1], shoulder_pos[1]], 
                                 [base_pos[2], shoulder_pos[2]], 
                                 color=self.colors['base'], linewidth=6)
                    
                    # Shoulder to elbow
                    self.ax.plot([shoulder_pos[0]], [shoulder_pos[1]], [shoulder_pos[2]], 'ko', markersize=8)
                    self.ax.plot([shoulder_pos[0], elbow_pos[0]], 
                                 [shoulder_pos[1], elbow_pos[1]], 
                                 [shoulder_pos[2], elbow_pos[2]], 
                                 color=self.colors['link1'], linewidth=4)
                    
                    # Elbow to end effector
                    self.ax.plot([elbow_pos[0]], [elbow_pos[1]], [elbow_pos[2]], 'ko', markersize=6)
                    self.ax.plot([elbow_pos[0], ee_pos[0]], 
                                 [elbow_pos[1], ee_pos[1]], 
                                 [elbow_pos[2], ee_pos[2]], 
                                 color=self.colors['link2'], linewidth=3)
                    
                    # End effector (gripper)
                    gripper_width = 1.5 if gripper_closed else 3
                    gripper_x = [ee_pos[0] - gripper_width/2, ee_pos[0] + gripper_width/2]
                    gripper_y = [ee_pos[1], ee_pos[1]]
                    gripper_z = [ee_pos[2], ee_pos[2]]
                    
                    self.ax.plot(gripper_x, gripper_y, gripper_z, 
                                 color=self.colors['gripper'], linewidth=2)
                
                def append_frame(self, angles, gripper_closed):
                    """
                    Add a new frame to the animation sequence.
                    
                    Args:
                        angles: Joint angles (base_angle, shoulder_angle, elbow_angle) in degrees
                        gripper_closed: Whether the gripper is closed
                    """
                    # Setup the scene and draw the robot
                    self.setup_3d_scene()
                    self.draw_robot(angles, gripper_closed)
                    
                    # Capture the frame
                    self.fig.canvas.draw()
                    self.frames.append(np.array(self.fig.canvas.renderer.buffer_rgba()))
                
                def update_piece_position(self, from_pos, to_pos, is_capture=False):
                    """Update the chess piece positions in the visualization."""
                    if from_pos in self.pieces:
                        piece_color, _ = self.pieces[from_pos]
                        self.pieces[from_pos] = (piece_color, True)  # Mark as captured/moved
                        
                        if is_capture and to_pos in self.pieces:
                            # Mark captured piece as captured
                            captured_color, _ = self.pieces[to_pos]
                            self.pieces[to_pos] = (captured_color, True)
                        
                        # Place moving piece in new position
                        self.pieces[to_pos] = (piece_color, False)
                
                def create_animation(self, interval=50, save_path='chess_robot_animation.gif'):
                    """
                    Create and save an animation from the stored frames.
                    
                    Args:
                        interval: Time between frames in milliseconds
                        save_path: Path to save the animation file (default is GIF format)
                        
                    Returns:
                        Animation object
                    """
                    # Create figure for animation
                    fig, ax = plt.subplots(figsize=(10, 8))
                    ax.axis('off')
                    
                    # Create a function to update the animation
                    def update(frame):
                        ax.clear()
                        ax.axis('off')
                        ax.imshow(self.frames[frame])
                        return [ax]
                    
                    # Create the animation
                    ani = FuncAnimation(fig, update, frames=len(self.frames), 
                                        interval=interval, blit=True)
                    
                    # Ensure the file has a .gif extension
                    if not save_path.lower().endswith('.gif'):
                        save_path = save_path.rsplit('.', 1)[0] + '.gif'
                    
                    # Save the animation as GIF (doesn't require ffmpeg)
                    try:
                        ani.save(save_path, writer='pillow', fps=10)
                        print(f"Animation saved as {save_path}")
                    except Exception as e:
                        print(f"Could not save animation: {e}")
                        print("Trying alternative method to save frames...")
                        try:
                            # Alternative: save individual frames
                            frames_dir = save_path.rsplit('.', 1)[0] + '_frames'
                            import os
                            os.makedirs(frames_dir, exist_ok=True)
                            
                            for i, frame in enumerate(self.frames):
                                frame_img = plt.figure(figsize=(10, 8))
                                frame_ax = frame_img.add_subplot(111)
                                frame_ax.axis('off')
                                frame_ax.imshow(frame)
                                frame_img.savefig(f"{frames_dir}/frame_{i:03d}.png")
                                plt.close(frame_img)
                            
                            print(f"Saved {len(self.frames)} individual frames to {frames_dir}/")
                        except Exception as e2:
                            print(f"Alternative saving method also failed: {e2}")
                    
                    plt.close(fig)
                    plt.close(self.fig)
                    
                    return ani
            
            # Create and attach the visualizer
            self.visualizer = RobotVisualizer(self)
            
        except ImportError as e:
            print(f"Visualization could not be enabled: {e}")
            print("Make sure matplotlib and numpy are installed.")
    
    def save_visualization(self, filename='chess_robot_animation.gif'):
        """
        Save the visualization animation to a file.
        
        Args:
            filename: Output filename for the animation (default is GIF format)
        """
        if hasattr(self, 'visualizer'):
            self.visualizer.create_animation(save_path=filename)
        else:
            print("Visualization is not enabled.")
    
    def update_visualization_pieces(self, from_pos, to_pos, is_capture=False):
        """
        Update the chess piece positions in the visualization.
        
        Args:
            from_pos: Starting position in algebraic notation
            to_pos: Target position in algebraic notation
            is_capture: Whether this move is a capture
        """
        if hasattr(self, 'visualizer'):
            self.visualizer.update_piece_position(from_pos, to_pos, is_capture)

def Setup():
    # Create robot with default parameters
    ser = setup_serial()
    robot = ChessRobot()

# # Example usage:
# if __name__ == "__main__":
#     # Create robot with default parameters
#     ser = setup_serial()
#     robot = ChessRobot()
#     robot.setup_visualization()
    
#     # Perform calibration
#     # robot.calibrate()
    
#     # Example moves
#     print("\n=== Example: Moving a pawn from e2 to e4 ===")
#     robot.pick_and_place_piece('e2', 'e4')
    
#     print("\n=== Example: Knight captures a pawn ===")
#     # robot.capture_piece('g1', 'f3')
    
#     # Print final status
#     print("\nRobot returned to home position and ready for next command.")
#     robot.save_visualization('chess_robot_demo.gif')