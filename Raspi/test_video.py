from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import apriltag

#options = apriltag.DetectorOptions(families='tag16h5',
#                                    border=1,
#                                    nthreads=4,
#                                    quad_decimate=1.0,
#                                    quad_blur=0.0,
#                                    refine_edges=True,
#                                    refine_decode=False,
#                                    refine_pose=False,
#                                    debug=False,
#                                    quad_contours=True)

detector = apriltag.Detector()

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array

    #res = detector.detect(image)
    cv2.imshow("Frame", image)
    #print(res)
    key = cv2.waitKey(1) & 0xFF

    rawCapture.truncate(0)

    if key == ord("q"):
        break
