# importing libraries
import cv2
import imutils
import numpy as np


def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    # channel_count = img.shape[2]
    match_mask_color = 255
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def drow_the_lines(img, lines):
    x1vals = []
    x2vals = []
    y1vals = []
    y2vals = []
    mvals = []
    height = img.shape[0]
    width = img.shape[1]
    m = 1000
    img = np.copy(img)
    blank_image = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)

    if lines is None:
        return img
    else:
        for line in lines:
            for x1, y1, x2, y2 in line:
                m = (y2-y1) / (x2-x1)
                #if m > 2 or m < -2:  # Use slopes to remove stuff
                cv2.line(blank_image, (x1, y1), (x2, y2), (0, 255, 0), thickness=2)
                if len(lines) >= 2:
                    #if m > 2 or m < -2:  # Use slopes to remove stuff
                    mvals.append(m)

        img = cv2.addWeighted(img, 0.8, blank_image, 1, 0.0)

    # Calculate averages
    avg_m = sum(mvals) / len(mvals) if len(mvals)>0 else 0.001
    # if avg_m == 0:
    #     avg_m = 0.001
    # if avg_m >= 10000:
    #     avg_m = 10000

    img = np.copy(img)
    blank_image2 = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    cv2.line(blank_image2, (int(width/2), int(height/2)),(int((width/2 + 10/avg_m)), int((height/2 + 20))), color=(255, 0, 0), thickness=2)
    img = cv2.addWeighted(img, 0.8, blank_image2, 1, 0.0)

    x1vals = []
    y1vals = []
    x2vals = []
    y2vals = []

    return img


def process_image(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    height = image.shape[0]
    width = image.shape[1]
    region_of_interest_vertices = [
        (0, height),
        (width/2, height/2 +10),
        (width, height)
    ]
    # region_of_interest_vertices = [
    #     (100, height),
    #     (width/2 - 100, height/1.5),
    #     (width/2 + 100, height/1.5),
    #     (width - 100, height)
    # ]
    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    canny_image = cv2.Canny(gray_image, 25, 200)
    cropped_image = region_of_interest(canny_image,
                                       np.array([region_of_interest_vertices], np.int32),)
    lines = cv2.HoughLinesP(cropped_image,
                            rho=2,
                            theta=np.pi/180,
                            threshold=50,
                            lines=np.array([]),
                            minLineLength=100,
                            maxLineGap=10)

    image_with_lines = drow_the_lines(image, lines)
    # return cropped_image
    image_with_lines = cv2.cvtColor(image_with_lines, cv2.COLOR_RGB2BGR)
    return image_with_lines


# Create a VideoCapture object and read from input file
cap = cv2.VideoCapture('pool2.mp4')
#cap = cv2.VideoCapture(0)

# Check if camera opened successfully
if (cap.isOpened() is False):
    print("Error opening video file")

# Read until video is completed
while (cap.isOpened()):

    # Capture frame-by-frame
    ret, frame = cap.read()

    frame = cv2.resize(frame, (640, 480))

    fnew = process_image(frame)


    if ret is True:
        # Display the resulting frame
        cv2.imshow('Frame', fnew)

    # Press Q on keyboard to exit
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

# Break the loop
    else:
        break

# When everything done, release
# the video capture object
cap.release()

# Closes all the frames
cv2.destroyAllWindows()
