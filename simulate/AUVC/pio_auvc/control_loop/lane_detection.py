import cv2
import numpy as np

def region_of_interest(img, vertices):
    """
    Apply an image mask to only show the region of interest (ROI).
    Args:
        img (numpy.ndarray): The input image.
        vertices (numpy.ndarray): The vertices of the polygon defining the ROI.
    Returns:
        numpy.ndarray: The masked image showing only the ROI.
    """
    mask = np.zeros_like(img)  # Create a mask of zeros (black image)
    cv2.fillPoly(mask, vertices, 255)  # Fill the ROI with white
    return cv2.bitwise_and(img, mask)  # Apply mask to the image

def detect_lanes(frame):
    """
    Detect lanes in the input frame using edge detection and Hough Transform.
    Args:
        frame (numpy.ndarray): The input image frame from the video.
    Returns:
        numpy.ndarray: The frame with detected lane lines drawn.
    """
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur to reduce noise and smoothen the image
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Apply Canny edge detector to find edges in the image
    # Parameters: lower and upper thresholds for edge detection
    edges = cv2.Canny(blur, 50, 150)  # You can tune these thresholds

    # Define region of interest (ROI) vertices
    height, width = edges.shape
    # Adjust these points based on the lane positions in your images
    roi_vertices = np.array([[(0, height), (width // 2, height // 2), (width, height)]], dtype=np.int32)
    
    # Apply the ROI mask to the edges image
    roi_edges = region_of_interest(edges, roi_vertices)

    # Detect lines using Hough Line Transform
    # Parameters:
    # 1. rho: Resolution of the accumulator in pixels (1 means 1 pixel)
    # 2. theta: Angle resolution in radians (np.pi / 180 means 1 degree)
    # 3. threshold: Minimum number of intersections in the accumulator to detect a line
    # 4. minLineLength: Minimum length of a line to be detected
    # 5. maxLineGap: Maximum allowed gap between points on the same line
    lines = cv2.HoughLinesP(roi_edges, 1, np.pi / 180, 50, minLineLength=100, maxLineGap=50)
    # Adjust minLineLength and maxLineGap to filter out shorter or fragmented lines

    # Draw the detected lines on the original frame
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)  # Draw lines in green with thickness 3

    return frame

def main():
    # Open video capture (0 for default camera, or filename for video file)
    cap = cv2.VideoCapture(0)  # Replace 0 with your video file path if needed

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        lane_detected_frame = detect_lanes(frame)
        cv2.imshow('Lane Detection', lane_detected_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

