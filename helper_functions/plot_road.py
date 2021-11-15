import matplotlib.pyplot as plt
import numpy as np
import cv2


def canny(iamge):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blur, 50, 150)
    return canny


def region_of_interest(image):
    height = image.shape[0]
    width = image.shape[1]
    mask = np.zeros_like(image)

    # this the triage of the road
    
    polygons = np.array([[(2, 62),  (125, 51), (55, 34)]], np.int32)

    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image


image = cv2.imread(r"image_original_1_1_33.jpeg")
lane_image = np.copy(image)
image_canny = canny(lane_image)
image_traingle = region_of_interest(image_canny)
# plt.imshow(image_canny)
plt.imshow(image_traingle)
plt.show()
# cv2.imshow("result", region_of_interest(image_canny))
# cv2.waitKey(0)
