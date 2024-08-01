import cv2
import numpy as np
import os
def region_of_interest(img, vertices):
    """
    Apply a mask to keep only the region of interest defined by vertices.
    """
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    return cv2.bitwise_and(img, mask)

def draw_lines(img, lines):
    """
    Draw lines on the image.
    """
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 3)

def process_frame(frame):
    """
    Process each frame to detect lanes.
    """
    height, width = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    
    # Define the region of interest
    vertices = np.array([[
        (0, height),
        (width / 2, height / 2),
        (width, height)
    ]], dtype=np.int32)
    
    roi = region_of_interest(edges, vertices)
    
    # Detect lines in the ROI
    lines = cv2.HoughLinesP(roi, 1, np.pi / 180, 50, minLineLength=100, maxLineGap=50)
    
    # Draw detected lines on the original frame
    draw_lines(frame, lines)
    
    return frame

def main():
    # Open video capture from USB camera (usually /dev/video0)
    #cap = cv2.VideoCapture(0)  # Change to the correct index if you have multiple cameras
    gstreamer_pipeline = (
        "udpsrc port=5000 ! application/x-rtp, encoding-name=JPEG ! rtpjpegdepay ! jpegdec ! videoconvert ! appsink drop=true max-buffers=1"
    )
    #print(f"Using GStreamer pipeline: {gstreamer_pipeline}")
    #os.environ['GST_DEBUG'] = '3'
    # Open video capture from GStreamer pipeline
    #print(cv2.CAP_GSTREAMER)
    cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Error: Could not open video capture.")
        return
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        processed_frame = process_frame(frame)
        
        # Display the result
        cv2.imshow('Lane Detection', processed_frame)
        
        # Exit loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release video capture and close windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

