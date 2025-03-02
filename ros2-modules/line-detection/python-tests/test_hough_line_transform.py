import cv2
import numpy as np


def convert_to_gray(img):
    # Convert the img to grayscale
    gray = cv2.cvtColor(test_img, cv2.COLOR_BGR2GRAY)
    return gray


def convert_to_edges(gray_img):
    # Apply edge detection method on the image
    edges = cv2.Canny(gray_img, 50, 150, apertureSize=3)
    return edges


def construct_houghlines(original_img, edges_img):
    lines = cv2.HoughLinesP(edges_img, 1, np.pi/180, 50)
    for line in lines:
        x1,y1, x2,y2 = line[0]
        cv2.line(original_img, (x1,y1), (x2,y2), (0,255,0))

    return original_img


if __name__ == '__main__':
    test_img = cv2.imread("./images/road-example.jpg")

    gray = convert_to_gray(test_img)
    edges = convert_to_edges(gray)
    lines = construct_houghlines(original_img=test_img, edges_img=edges)

    cv2.imshow("Original", test_img)
    cv2.imshow("Edges", edges)
    cv2.imshow("Lines", lines)

    while(cv2.waitKey(10) != ord('q')):
        continue
    cv2.destroyAllWindows()
