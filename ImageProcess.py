import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the image
image = cv2.imread("map.jpeg", cv2.IMREAD_GRAYSCALE)

# Apply Gaussian Blur to reduce noise
blurred = cv2.GaussianBlur(image, (5, 5), 0)

# Apply adaptive thresholding to binarize the image
_, binary = cv2.threshold(blurred, 10, 255, cv2.THRESH_BINARY_INV)

# Apply morphological operations to remove small noise and close gaps
kernel = np.ones((3, 3), np.uint8)
processed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)

# Find contours of the walls
contours, _ = cv2.findContours(processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Create a blank image to fill the walls
filled_walls = np.zeros_like(image)

# Fill the detected wall contours
cv2.drawContours(filled_walls, contours, -1, (255, 255, 255), thickness=cv2.FILLED)

# Show the results
plt.figure(figsize=(10, 5))
plt.subplot(1, 2, 1), plt.imshow(image, cmap="gray"), plt.title("Original Image")
plt.subplot(1, 2, 2), plt.imshow(filled_walls, cmap="gray"), plt.title("Filled Walls")
plt.show()

# Save the binary filled wall image
cv2.imwrite("filled_walls.png", filled_walls)