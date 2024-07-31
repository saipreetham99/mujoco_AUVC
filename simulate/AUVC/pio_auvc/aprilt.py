import cv2
import apriltag
import numpy as np

def calculate_pose(corners, camera_matrix, dist_coeffs):
    """
    Calculate the pose of the AprilTag.
    """
    # Assuming the tag is on the z = 0 plane
    tag_size = 0.165  # Size of the AprilTag in meters, adjust as needed

    # Prepare the object points based on tag size
    obj_points = np.array([
        [-tag_size / 2, -tag_size / 2, 0],
        [tag_size / 2, -tag_size / 2, 0],
        [tag_size / 2, tag_size / 2, 0],
        [-tag_size / 2, tag_size / 2, 0]
    ], dtype=np.float32)

    # Convert corners to numpy array
    img_points = np.array(corners, dtype=np.float32)

    # Solve the PnP problem to get rotation and translation vectors
    _, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)

    # Convert rotation vector to rotation matrix
    R, _ = cv2.Rodrigues(rvec)

    # Compute Euler angles from rotation matrix
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    # Convert to degrees
    x, y, z = np.degrees([x, y, z])

    return tvec.flatten(), (x, y, z)

def main():
    # Camera intrinsic parameters (adjust these for your camera)
    camera_matrix = np.array([
        [800, 0, 320],
        [0, 800, 240],
        [0, 0, 1]
    ], dtype=np.float32)

    dist_coeffs = np.zeros((4, 1), dtype=np.float32)

    # Initialize the camera
    cap = cv2.VideoCapture(0)  # Use your camera index

    # Initialize the AprilTag detector
    detector = apriltag.Detector()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(gray)

        for tag in tags:
            corners = tag.corners
            tvec, rpy = calculate_pose(corners, camera_matrix, dist_coeffs)

            # Draw the tag and its pose
            for i in range(4):
                p1 = (int(corners[i][0]), int(corners[i][1]))
                p2 = (int(corners[(i + 1) % 4][0]), int(corners[(i + 1) % 4][1]))
                cv2.line(frame, p1, p2, (0, 255, 0), 2)

            # Display pose
            text = f"Pos: ({tvec[0]:.2f}, {tvec[1]:.2f}, {tvec[2]:.2f}), Yaw: {rpy[0]:.2f}, Pitch: {rpy[1]:.2f}, Roll: {rpy[2]:.2f}"
            cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.imshow('AprilTag Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
