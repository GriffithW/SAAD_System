# import io
# import time
# import picamera
# from base_camera import BaseCamera


# class Camera(BaseCamera):
#     @staticmethod
#     def frames():
#         with picamera.PiCamera() as camera:
#             # let camera warm up
#             time.sleep(2)

#             stream = io.BytesIO()
#             for _ in camera.capture_continuous(stream, 'jpeg',
#                                                  use_video_port=True):
#                 # return current frame
#                 stream.seek(0)
#                 yield stream.read()

#                 # reset stream for next frame
#                 stream.seek(0)
#                 stream.truncate()
import io
import time
import numpy as np
from picamera2 import Picamera2, Preview
from base_camera import BaseCamera
import cv2  # OpenCV for JPEG encoding

class Camera(BaseCamera):
    @staticmethod
    def frames():
        # Initialize the camera
        picam2 = Picamera2()
        
        # Configure the camera (adjust resolution if needed)
        config = picam2.create_still_configuration(main={"size": (640, 480)})
        picam2.configure(config)
        
        picam2.start()
        time.sleep(2)  # Let the camera warm up

        while True:
            frame = picam2.capture_array("main")  # Capture raw image
            _, jpeg = cv2.imencode(".jpg", frame)  # Convert to JPEG

            yield jpeg.tobytes()  # Yield as a byte stream
