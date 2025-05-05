import cv2
import numpy as np
import json

# This code takes a picture of the board, identifies the contours of the squares, and labels them

###################### TAKES PICTURE OF BOARD ######################################################################
def capture_and_save():
    cap = cv2.VideoCapture(1)  # Open default webcam (change to 1, 2, etc., if using a different camera)
    
    if not cap.isOpened():
        print("Error: Could not access webcam")
        return

    ret, frame = cap.read()  # Capture a frame
    cap.release()  # Release the webcam

    if ret:
        cv2.imwrite("captured_chessboard2.jpg", frame)  # Save the captured image
        print("Image saved as captured_chessboard.jpg")
    else:
        print("Error: Could not capture image")

capture_and_save()

###################### DETECTS BOARD SQUARES ######################################################################

def detect_chessboard_squares(image_path):
    # Load image
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Chessboard parameters (7x7 inner corners for 8x8 squares)
    pattern_size = (7, 7)
    found, corners = cv2.findChessboardCorners(gray, pattern_size, None)

    if not found:
        print("Chessboard pattern not detected")
        return

    # Refine corner locations
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    # Create ideal inner grid points (1-7 in both axes)
    ideal_inner = np.zeros((np.prod(pattern_size), 2), dtype=np.float32)
    ideal_inner[:, :] = np.mgrid[1:8, 1:8].T.reshape(-1, 2)

    # Calculate homography from ideal to detected corners
    H, _ = cv2.findHomography(ideal_inner, corners)

    # Define outer chessboard corners in ideal coordinates
    ideal_outer = np.array([[[0, 0]], [[8, 0]], [[0, 8]], [[8, 8]]], dtype=np.float32)
    original_outer = cv2.perspectiveTransform(ideal_outer, H)

    # Create destination points for perspective warp
    dst_size = 800
    dst_points = np.array([[0, 0], [dst_size, 0],
                           [0, dst_size], [dst_size, dst_size]], dtype=np.float32)

    # Calculate final homography matrix
    H_final, _ = cv2.findHomography(original_outer, dst_points)

    # Calculate inverse homography for back transformation
    H_inv = np.linalg.inv(H_final)

    # Generate and transform square coordinates
    square_size = dst_size // 8
    squares = {}

    files = "abcdefgh"
    ranks = "87654321"  

    # Reverse the order to match OpenCV's detection (Top-Right to Bottom-Left)
    for col in range(8):  # Move from right to left (h → a)
        for row in range(8):  # Move from top to bottom (1 → 8)
            # Warped image coordinates, bottom left, top left, top right, bottom right
           
            pts = np.array([[[ 
                col * square_size, row * square_size],
                [(col+1) * square_size, row * square_size],
                [(col+1) * square_size, (row+1) * square_size],
                [col * square_size, (row+1) * square_size]
            ]], dtype=np.float32)

            square_name = f"{files[col]}{ranks[row]}"  # Assign name based on adjusted order

            # Transform to original image coordinates
            original_pts = cv2.perspectiveTransform(pts, H_inv)
            print(f"{square_name}: {original_pts}")

            squares[square_name] = original_pts.reshape(-1, 2).tolist()


    # Save output to JSON file
    with open("new_boxes.json", "w") as file:
        json.dump(squares, file, indent=4)

    print("Saved chessboard squares to new_boxes.json")

    # Draw detected squares on original image
    for square_name, square in squares.items():
        pts = np.array(square, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(image, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

        # Label squares in the image
        center_x = int(sum(p[0] for p in square) / 4)
        center_y = int(sum(p[1] for p in square) / 4)
        cv2.putText(image, square_name, (center_x - 15, center_y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

    # Display results
    cv2.imshow('Chessboard Detection', image)

    # **Fix: Close after 5 seconds (or press any key)**
    cv2.waitKey(5000)  # Wait for 5 seconds
    cv2.destroyAllWindows()  # Ensure all OpenCV windows close properly

# Usage example
detect_chessboard_squares('captured_chessboard.jpg')
