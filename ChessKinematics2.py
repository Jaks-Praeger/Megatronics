import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import os
import math
from IPython.display import HTML
import time
import serial

# This is an improved visualization class to fix the blank GIF issue
class ImprovedRobotVisualizer:
    def __init__(self, robot):
        self.robot = robot
        
        # Store robot parameters for reference
        self.base_height = robot.base_height
        self.link1_length = robot.link1_length
        self.link2_length = robot.link2_length
        self.chess_square_size = robot.chess_square_size
        self.board_height = robot.board_height
        self.board_origin_x = robot.board_origin_x
        self.board_origin_y = robot.board_origin_y
        
        # Create figure and ax for 3D visualization
        plt.ioff()  # Turn off interactive mode to avoid displaying plots
        self.fig = plt.figure(figsize=(12, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Store raw data for animation instead of rendered frames
        self.animation_data = []
        self.pieces = {}  # Chess pieces on the board
        
        # Colors for visualization
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
    
    def update_piece_position(self, from_pos, to_pos, is_capture=False):
        """Update the chess piece positions in the visualization."""
        if from_pos in self.pieces:
            piece_color, _ = self.pieces[from_pos]
            self.pieces[from_pos] = (piece_color, True)  # Mark as captured/moved
            
            if is_capture and to_pos in self.pieces:
                # Mark captured piece as captured
                captured_color, _ = self.pieces[to_pos]
                self.pieces[to_pos] = (captured_color, True)
            
            # Place moving piece in new position (only if to_pos is not None)
            if to_pos is not None:
                self.pieces[to_pos] = (piece_color, False)
    
    def append_frame(self, angles, gripper_closed):
        """
        Store robot state for a frame instead of rendering immediately
        """
        # Store the data needed to render the frame later
        self.animation_data.append({
            'angles': angles.copy() if isinstance(angles, list) else list(angles),
            'gripper_closed': gripper_closed,
            'pieces': {k: v for k, v in self.pieces.items()}  # Make a copy of the current piece state
        })
    
    def get_joint_positions(self, angles):
        """Calculate positions of all joints for visualization."""
        base_angle, shoulder_angle, elbow_angle = angles
        
        # Base position
        base_pos = (0, 0, 0)
        
        # Shoulder position
        shoulder_pos = (0, 0, self.base_height)
        
        # Convert angles to radians
        base_rad = math.radians(base_angle)
        shoulder_rad = math.radians(shoulder_angle)
        # FIXED: Use elbow angle directly as the interior angle between links
        elbow_rad = math.radians(elbow_angle)
        
        # Calculate elbow position
        elbow_x = self.link1_length * math.cos(shoulder_rad) * math.cos(base_rad)
        elbow_y = self.link1_length * math.cos(shoulder_rad) * math.sin(base_rad)
        elbow_z = self.base_height + self.link1_length * math.sin(shoulder_rad)
        elbow_pos = (elbow_x, elbow_y, elbow_z)
        
        # FIXED: Calculate the absolute angle of the second link
        # The second link angle is relative to the horizontal plane
        # It's determined by the shoulder angle and the interior elbow angle
        second_link_angle = shoulder_rad - (math.pi - elbow_rad)
        
        # Calculate end effector position
        ee_x = elbow_x + self.link2_length * math.cos(second_link_angle) * math.cos(base_rad)
        ee_y = elbow_y + self.link2_length * math.cos(second_link_angle) * math.sin(base_rad)
        ee_z = elbow_z + self.link2_length * math.sin(second_link_angle)
        ee_pos = (ee_x, ee_y, ee_z)
        
        return base_pos, shoulder_pos, elbow_pos, ee_pos
    
    def draw_robot(self, angles, gripper_closed=False):
        """Draw the robot at the given joint angles."""
        # Get joint positions
        base_pos, shoulder_pos, elbow_pos, ee_pos = self.get_joint_positions(angles)
        
        # Draw base 
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
    
    def draw_chessboard(self):
        """Draw the chess board in 3D space."""
        # Create the chess board grid
        for i in range(8):  # files a-h
            for j in range(8):  # ranks 1-8
                x = self.board_origin_x + i * self.chess_square_size
                y = self.board_origin_y + j * self.chess_square_size
                z = self.board_height
                
                # Determine square color
                color = self.colors['chess_light'] if (i + j) % 2 == 0 else self.colors['chess_dark']
                
                # Create a square as a rectangular plane
                xx, yy = np.meshgrid([x, x + self.chess_square_size], 
                                     [y, y + self.chess_square_size])
                zz = np.ones_like(xx) * z
                
                self.ax.plot_surface(xx, yy, zz, color=color, edgecolor='black', linewidth=0.5, alpha=0.8)
    
    def draw_chess_piece(self, position, piece_color='white', captured=False):
        """Draw a chess piece at the specified position."""
        # Skip drawing if position is None or the piece is captured
        if position is None or captured:
            return
            
        if isinstance(position, str):
            x, y = self.robot.algebraic_to_coordinate(position)
        else:
            x, y = position
            
        z = self.board_height
        
        # Piece height
        piece_height = self.chess_square_size * 0.8
        
        # Create a simple cylinder to represent the piece
        r = self.chess_square_size * 0.3  # Radius
        theta = np.linspace(0, 2*np.pi, 20)
        z_vals = np.linspace(z, z + piece_height, 10)
        theta_grid, z_grid = np.meshgrid(theta, z_vals)
        
        x_grid = x + r * np.cos(theta_grid)
        y_grid = y + r * np.sin(theta_grid)
        
        color = self.colors['piece_white'] if piece_color == 'white' else self.colors['piece_black']
        self.ax.plot_surface(x_grid, y_grid, z_grid, color=color, alpha=0.9)
    
    def render_frame(self, frame_data):
        """Render a complete frame based on stored data"""
        # Clear the previous frame
        self.ax.clear()
        
        # Draw the chess board
        self.draw_chessboard()
        
        # Draw chess pieces based on their state for this frame
        for pos, (color, captured) in frame_data['pieces'].items():
            if not captured:
                self.draw_chess_piece(pos, color, captured)
        
        # Draw the robot
        self.draw_robot(frame_data['angles'], frame_data['gripper_closed'])
        
        # Set the axis properties
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.set_zlabel('Z (cm)')
        
        # Set view limits
        max_dim = max(self.board_origin_x + 8*self.chess_square_size, 
                      self.board_origin_y + 8*self.chess_square_size,
                      self.base_height + self.link1_length + self.link2_length)
        
        # Add some margin
        margin = 10
        # self.ax.set_xlim([-margin, max_dim + margin])
        # self.ax.set_ylim([-margin, max_dim + margin])
        # self.ax.set_zlim([0, max_dim])
        self.ax.set_xlim([self.board_origin_x - margin, self.board_origin_x + 8*self.chess_square_size + margin])
        self.ax.set_ylim([self.board_origin_y - margin, self.board_origin_y + 8*self.chess_square_size + margin])
        self.ax.set_zlim([0, self.base_height + self.link1_length + self.link2_length])
        
        # Set an isometric-like view
        self.ax.view_init(elev=30, azim=45)
        
        # Set the title
        self.ax.set_title('Chess Robot 3D Visualization')
    
    def save_individual_frames(self, output_dir="chess_robot_frames"):
        """Save individual frames as png files"""
        os.makedirs(output_dir, exist_ok=True)
        
        print(f"Generating {len(self.animation_data)} frames...")
        for i, frame_data in enumerate(self.animation_data):
            print(f"Rendering frame {i+1}/{len(self.animation_data)}", end='\r')
            
            # Clear and render the frame
            self.render_frame(frame_data)
            
            # Save the frame as a PNG
            filename = os.path.join(output_dir, f"frame_{i:04d}.png")
            self.fig.savefig(filename, dpi=100, bbox_inches='tight')
        
        print(f"\nSaved {len(self.animation_data)} frames to {output_dir}/")
        return output_dir
    
    def create_animation(self, filename="chess_robot_animation.gif", method="imageio", fps=10):
        """
        Create an animation from the stored frames
        
        Args:
            filename: Output filename
            method: Animation method ('imageio' or 'matplotlib')
            fps: Frames per second
        """
        # Make sure output is a gif
        if not filename.lower().endswith('.gif'):
            filename = filename.rsplit('.', 1)[0] + '.gif'
        
        print("Starting animation creation process...")
        
        if method == "matplotlib":
            # Try the matplotlib animation approach
            try:
                print("Creating animation using matplotlib...")
                plt.ioff()
                fig, ax = plt.subplots(figsize=(10, 8))
                ax.axis('off')
                
                # Draw the first frame to set up the plot
                self.render_frame(self.animation_data[0])
                self.fig.canvas.draw()
                img = np.array(self.fig.canvas.renderer.buffer_rgba())
                
                # Set up the plot
                plot = ax.imshow(img)
                
                def update(frame_idx):
                    self.render_frame(self.animation_data[frame_idx])
                    self.fig.canvas.draw()
                    plot.set_array(np.array(self.fig.canvas.renderer.buffer_rgba()))
                    return [plot]
                
                # Create the animation
                ani = FuncAnimation(fig, update, frames=len(self.animation_data), 
                                    interval=1000/fps, blit=True)
                
                # Save the animation
                ani.save(filename, writer='pillow', fps=fps)
                plt.close(fig)
                plt.close(self.fig)
                print(f"Animation saved as {filename}")
                return True
                
            except Exception as e:
                print(f"Matplotlib animation failed: {e}")
                print("Trying alternative method...")
        
        # If we reach here, use imageio method
        try:
            # First save individual frames
            frames_dir = self.save_individual_frames(output_dir=f"{filename}_frames")
            
            # Use imageio to create the gif
            try:
                import imageio
                
                # Create the gif using imageio
                print("Creating GIF using imageio...")
                with imageio.get_writer(filename, mode='I', fps=fps) as writer:
                    for i in range(len(self.animation_data)):
                        frame_path = os.path.join(frames_dir, f"frame_{i:04d}.png")
                        image = imageio.imread(frame_path)
                        writer.append_data(image)
                
                print(f"Animation saved as {filename}")
                return True
                
            except ImportError:
                print("imageio not found, using PIL instead...")
                
                # Use PIL to create the gif
                try:
                    from PIL import Image
                    
                    frames = []
                    for i in range(len(self.animation_data)):
                        frame_path = os.path.join(frames_dir, f"frame_{i:04d}.png")
                        frames.append(Image.open(frame_path))
                    
                    # Save the gif
                    frames[0].save(
                        filename,
                        save_all=True,
                        append_images=frames[1:],
                        optimize=False,
                        duration=1000/fps,
                        loop=0
                    )
                    
                    print(f"Animation saved as {filename}")
                    return True
                    
                except Exception as e:
                    print(f"PIL animation failed: {e}")
                    print(f"Please check the individual frames in {frames_dir}/")
        
        except Exception as e:
            print(f"Animation creation failed: {e}")
            print("Falling back to saving individual frames...")
            self.save_individual_frames()
            
        return False

# This is a small utility function to replace the setup_visualization method in your ChessRobot class
def setup_improved_visualization(robot):
    """Set up the improved 3D visualization for the robot."""
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        import numpy as np
        
        print("Setting up improved 3D visualization...")
        
        # Create and attach the visualizer
        robot.visualizer = ImprovedRobotVisualizer(robot)
        
    except ImportError as e:
        print(f"Visualization could not be enabled: {e}")
        print("Make sure matplotlib and numpy are installed.")

class ChessRobot:
    def __init__(self, base_height=2.8448, link1_length=37.2872, link2_length=36.83, 
                 gripper_height=14.732, chess_square_size=4.07, board_height=2.5273,
                 board_origin_x=-19.1, board_origin_y=31.625, linkage_angle_deg=99.8, linkage_length=4.9784, home_position=(90, 90, 20)):
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
        self.z_clearance = 10.0 + self.gripper_height  # cm above pieces for safe movement

        self.sendActualCommands = False

        if self.sendActualCommands:
            self.ser = self.setup_serial()

        self.allowAngles = 0
        if not self.sendActualCommands:
            self.allowAngles = True

    def setup_serial(self, port='COM8', baud_rate=9600):
        """Set up serial connection to Arduino"""
        ser = serial.Serial(port, baud_rate, timeout=1)
        ser.setDTR(False)  # Prevents reset
        time.sleep(2)  # Allow connection to establish
        return ser

    def sendAngleArray(self, angles):
        """
        Send an array of 3 angles to Arduino
        
        Args:
            ser: Serial connection object
            angles: List of 3 angle values
        """
        # Read a line of data from the serial port
        while not self.allowAngles:
            while self.ser.in_waiting > 0:
                data = self.ser.readline().decode('utf-8').rstrip()

                if data == "Calibration complete":
                    self.allowAngles = 1
            
                # Print the received data
                print(data)

        # Format the three angles with comma separators
        data_string = f"{round(angles[0], 2), round(angles[1], 2), round(angles[2], 2), round(self.gripper_closed, 2)}\n"
        print(f"Printing from RPi: {data_string}\n")

        time.sleep(0.5)
        
        # Send the data
        if self.sendActualCommands:
            self.ser.write(data_string.encode())

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

        # Calculate the shoulder angle
        alpha = math.degrees(math.atan2(height_from_shoulder, r_xy))
        beta = math.degrees(math.acos((self.link1_length**2 + reach**2 - self.link2_length**2) / (2 * self.link1_length * reach)))
        pre_shoulder_angle = alpha + beta
        print(f"Pre-shoulder angle: {pre_shoulder_angle}")

        # with that, subtract the linkage contribution to the target position
        linkage_angle_rad = math.radians(self.linkage_angle_deg)
        pre_shoulder_angle_rad = math.radians(pre_shoulder_angle)
        linkage_x = self.linkage_length * math.cos(linkage_angle_rad + pre_shoulder_angle_rad)
        linkage_y = self.linkage_length * math.sin(linkage_angle_rad + pre_shoulder_angle_rad)
        print(f"Linkage contribution: {linkage_x}, {linkage_y}")

        r_xy -= linkage_x
        height_from_shoulder -= linkage_y
        reach = math.sqrt(r_xy**2 + height_from_shoulder**2)

        # Now recalculate the angles with the new target position
        alpha = math.degrees(math.atan2(height_from_shoulder, r_xy))
        beta = math.degrees(math.acos((self.link1_length**2 + reach**2 - self.link2_length**2) / (2 * self.link1_length * reach)))
        shoulder_angle = alpha + beta
        print(f"Actual shoulder angle: {shoulder_angle}")

        # Use the law of cosines to calculate the elbow angle
        cos_elbow = (self.link1_length**2 + self.link2_length**2 - reach**2) / (2 * self.link1_length * self.link2_length)
        # Clamp to valid range to avoid numerical errors
        cos_elbow = max(min(cos_elbow, 1.0), -1.0)  

        # FIXED: Return the actual interior angle at the elbow
        elbow_angle = math.degrees(math.acos(cos_elbow))

        return (base_angle, shoulder_angle, elbow_angle)

    def forward_kinematics(self, base_angle, shoulder_angle, elbow_angle):
        """
        Calculate the end-effector position given the joint angles.
        
        Args:
            base_angle: Base rotation angle in degrees
            shoulder_angle: Shoulder angle in degrees
            elbow_angle: Elbow angle in degrees (interior angle between links)
            
        Returns:
            tuple: (x, y, z) coordinates in cm
        """
        # Convert angles to radians
        base_rad = math.radians(base_angle)
        shoulder_rad = math.radians(shoulder_angle)
        elbow_rad = math.radians(elbow_angle)  # FIXED: Use elbow angle directly
        
        # Calculate the position of the elbow joint
        elbow_x = self.link1_length * math.cos(shoulder_rad) * math.cos(base_rad)
        elbow_y = self.link1_length * math.cos(shoulder_rad) * math.sin(base_rad)
        elbow_z = self.base_height + self.link1_length * math.sin(shoulder_rad)
        
        # FIXED: Calculate the absolute angle of the second link
        second_link_angle = shoulder_rad - (math.pi - elbow_rad)
        
        # Calculate the position of the end effector
        x = elbow_x + self.link2_length * math.cos(second_link_angle) * math.cos(base_rad)
        y = elbow_y + self.link2_length * math.cos(second_link_angle) * math.sin(base_rad)
        z = elbow_z + self.link2_length * math.sin(second_link_angle)
        
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
                # print(f"Moving to angles: Base={intermediate_angles[0]:.1f}°, "
                      # f"Shoulder={intermediate_angles[1]:.1f}°, Elbow={intermediate_angles[2]:.1f}°")
                self.send_to_motors(intermediate_angles)
                
                time.sleep(0.05)  # Simulate movement time
            
            # Append a frame to the visualization if visualizer is available
            if hasattr(self, 'visualizer'):
                self.visualizer.append_frame(intermediate_angles, self.gripper_closed)

            # Update current angles
            self.current_angles = intermediate_angles
            
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

    def return_to_home(self):
        """Move the robot back to its home position."""
        print("Returning to home position")
        self.move_joints(self.home_position)
        self.actuate_gripper(False)  # Open gripper

    def move_piece(self, from_pos, to_pos):
        """Helper function to move a piece and update visualization"""
        # Convert chess positions to coordinates
        from_x, from_y = self.algebraic_to_coordinate(from_pos)
        to_x, to_y = self.algebraic_to_coordinate(to_pos)
        
        # Calculate Z heights
        piece_top_z = self.board_height + self.z_clearance + 10
        hover_z = self.board_height + self.z_clearance + 10
        grip_z = self.board_height + self.z_clearance
        place_z = self.board_height + self.z_clearance
        
        # 1. Open gripper
        self.actuate_gripper(False)
        
        # 2. Move above the piece to pick
        self.move_to_position(from_x, from_y, hover_z)
        
        # 3. Lower to grip the piece
        self.move_to_position(from_x, from_y, grip_z, avoid_collisions=False)
        
        # 4. Close gripper to grab the piece
        self.actuate_gripper(True)
        
        # Update visualization piece state if enabled
        if hasattr(self, 'visualizer'):
            self.visualizer.update_piece_position(from_pos, to_pos)
        
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
        self.move_joints(self.home_position)

    def capture_piece(self, pos, captured_storage=(35, 35)):
        """Helper function to capture a piece and update visualization"""
        # Convert chess position to coordinates
        to_x, to_y = self.algebraic_to_coordinate(pos)
        store_x, store_y = captured_storage
        
        # 1. Open gripper
        self.actuate_gripper(False)
        
        # 2. Move to and pick up the captured piece
        self.move_to_position(to_x, to_y, self.board_height + self.chess_square_size * 0.3)
        self.actuate_gripper(True)
        
        # 3. Update visualization piece state if enabled
        if hasattr(self, 'visualizer'):
            self.visualizer.update_piece_position(pos, None, is_capture=True)
        
        # 4. Lift the captured piece
        self.move_to_position(to_x, to_y, self.board_height + self.chess_square_size * 0.7, avoid_collisions=False)
        
        # 5. Move the captured piece to storage area
        self.move_to_position(store_x, store_y, self.board_height + self.chess_square_size * 0.7)
        
        # 6. Place the captured piece
        self.move_to_position(store_x, store_y, self.board_height + 0.1, avoid_collisions=False)
        self.actuate_gripper(False)
        
        # 7. Lift up from storage area
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

def run_chess_robot_demo():
    # Create robot with visualization-friendly parameters
    # Note: Modified link lengths to create a better visualization
    robot = ChessRobot()
    
    # Set up improved visualization
    # setup_improved_visualization(robot)
    
    # 1. King's pawn opening (e2 to e4)
    # print("\nMoving pawn from e2 to e4")
    robot.move_piece('e1', 'e8')
    
    # 2. Knight to f3
    # print("\nMoving knight from g1 to f3")
    # robot.move_piece('g1', 'f3')
    
    # 3. Queen's pawn opening (d2 to d4)
    # print("\nMoving pawn from d2 to d4")
    # robot.move_piece('d2', 'd4')
    
    # 4. Knight captures on e4
    # print("\nCapturing with knight from f3 to e4")
    # robot.capture_piece('f3', 'e4')
    
    # Save the animation
    # print("\nSaving animation...")
    # if hasattr(robot, 'visualizer'):
    #     robot.visualizer.create_animation(filename="chess_robot_demo.gif", method="imageio", fps=10)

    print("\nDemo completed!")

if __name__ == "__main__":
    run_chess_robot_demo()