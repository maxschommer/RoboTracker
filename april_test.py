import apriltag
import cv2
img = cv2.imread('apriltagrobots_overlay.jpg', cv2.IMREAD_GRAYSCALE)
detector = apriltag.Detector()
result = detector.detect(img)
