import matplotlib.pylab as plt
import cv2
import numpy as np


def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    # channel_count = img.shape[2]
    match_mask_color = 255
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def drow_the_lines(img, lines):
    img = np.copy(img)
    blank_image = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(blank_image, (x1, y1), (x2, y2), (0, 255, 0), thickness=4)

        img = cv2.addWeighted(img, 0.8, blank_image, 1, 0.0)
    return img


image = cv2.imread('pool.jpg')
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

print(image.shape)
height = image.shape[0]
width = image.shape[1]
region_of_interest_vertices = [
    (150, height),
    (width/2, height/2 +10),
    (width-150, height)
]
gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
canny_image = cv2.Canny(gray_image, 25, 200)
cropped_image = region_of_interest(gray_image,
                                   np.array([region_of_interest_vertices], np.int32),)
mask = cv2.inRange(cropped_image, 25, 255)

result = cv2.bitwise_and(cropped_image, cropped_image, mask=mask)
lines = cv2.HoughLinesP(result,
                        rho=2,
                        theta=np.pi/360,
                        threshold=50,
                        lines=np.array([]),
                        minLineLength=100,
                        maxLineGap=25)


image_with_lines = drow_the_lines(image, lines)

# for line in lines:
#     for x1, y1, x2, y2 in line:
#         print("x1:", x1, "y1:", y1, "x2:", x2, "y2:", y2)

plt.imshow(image_with_lines)
plt.show()
