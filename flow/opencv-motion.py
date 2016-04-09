# http://picamera.readthedocs.org/en/release-1.10/api_array.html#pimotionanalysis

import numpy as np
import picamera
import picamera.array
import time

class DetectMotion(picamera.array.PiMotionAnalysis):
    def analyse(self, a):
        a = np.sqrt(
            np.square(a['x'].astype(np.float)) +
            np.square(a['y'].astype(np.float))
            ).clip(0, 255).astype(np.uint8)
        # If there're more than 10 vectors with a magnitude greater
        # than 60, then say we've detected motion
        greater_than = (a > 60).sum()
        if greater_than > 10:
            t = time.time()
            print('{0}: Motion detected! ({1})'.format(int(t), greater_than))

with picamera.PiCamera() as camera:
    with DetectMotion(camera) as output:
        camera.resolution = (640, 480)
        camera.start_recording(
              '/dev/null', format='h264', motion_output=output)
        camera.wait_recording(50)
        camera.stop_recording()
