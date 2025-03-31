import cv2
cap = cv2.VideoCapture('/dev/video1')
if cap.isOpened():
    print("Device opened successfully")
else:
    print("Failed to open device")
