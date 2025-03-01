import cv2
import numpy as np

# Load the image
image = cv2.imread(r"C:\Users\daphn\OneDrive\Documents\Boulder Spring 2025\Embedded\Megatronics\IMG_5580.jpg", cv2.IMREAD_GRAYSCALE)  # Load as grayscale

if image is None:
    print(f"Error: Could not load image")
else:
    print("Image loaded successfully!")

scale_factor = 0.04  # Reduce image size to 10%
small_image = cv2.resize(image, (0, 0), fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_AREA)

# Apply thresholding to ensure binary image
_, binary = cv2.threshold(small_image, 128, 255, cv2.THRESH_BINARY)  # 128 is the mid-point

# Get dimensions
height, width = binary.shape

print(f"Image dimensions: {width}x{height}")
# Convert to ASCII representation
maze_text = ""
for y in range(height):
    for x in range(width):
        if binary[y, x] == 255:  # White (walls)
            maze_text += "#"
        else:  # Black (floor)
            maze_text += " "
    maze_text += "\n"  # New line after each row

# print(maze_text)

# Save to a text file
with open(r"C:\Users\daphn\OneDrive\Documents\Boulder Spring 2025\Embedded\Megatronics\mazeActual.txt", "w") as f:
    f.write(maze_text)

print("Maze saved to mazeActual.txt")
