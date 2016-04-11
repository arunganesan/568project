import cv2
import numpy as np
from numpy import linalg as LA
import numpy as np
import imutils
from WarpImages import *

def plotcov2d (image, center, covariance, color=(255, 0, 0), nSigma=3):
  w, v = LA.eig(covariance)
  lam1 = w[0]; lam2 = w[1];
  v1 = v[:,0]; v2 = v[:,1];
  if v1[0] == 0:
    angle = 90
  else:
    angle = -np.math.degrees(np.math.atan(v1[1]/v1[0]));
  
  width = nSigma * np.sqrt(lam1);
  height = nSigma * np.sqrt(lam2);
  
  box = (center, (width, height), angle);
  
  cv2.circle(image, center, 1, color, -1);
  cv2.ellipse(image, box, color)



class VideoCamera(object):
    def __init__(self):
        # Using OpenCV to capture from device 0. If you have trouble capturing
        # from a webcam, comment the line below out and use a video file
        # instead.    def __
        self.video0 = cv2.VideoCapture(0)
        self.video0.set(3, 1920)
        self.video0.set(4, 1080)
        self.video0.set(5, 5)

        self.video1 = cv2.VideoCapture(1)
        self.video1.set(3, 1920)
        self.video1.set(4, 1080)
        self.video1.set(5, 5)

        self.video2 = cv2.VideoCapture(2)
        self.video2.set(3, 1920)
        self.video2.set(4, 1080)
        self.video2.set(5, 5)        
        
        # If you decide to use video.mp4, you must have this file in the folder
        # as the main.py.
        #self.video = cv2.VideoCapture('video.mp4')
    
    def __del__(self):
        self.video0.release()
        self.video1.release()
        self.video2.release()
    
    def get_frame(self):
        import time
        #t0 = time.time()
        success, image0 = self.video0.read()
        success, image1 = self.video1.read()
        success, image2 = self.video2.read()
        #print 'Reading: {}'.format(time.time() - t0)
        #t0 = time.time()
        
        #cv2.imshow('string!', image)       
        #cv2.waitKey(0)
        # We are using Motion JPEG, but OpenCV defaults to capture raw images,
        # so we must encode it into JPEG in order to correctly display the
        # video stream.

        
        image = computeStitch(image0, image1, image2)
        #print 'Stitching:{}'.format(time.time() - t0)        

        #print image.shape
        center = (100, 100)
        covariance = np.array([[500, 100], [100, 500]]);    
        
        #plotcov2d(image, center, covariance, color=(255, 0, 0), nSigma=1)
        #plotcov2d(image, center, covariance, color=(0, 255, 0), nSigma=4)
        #plotcov2d(image, center, covariance, color=(0, 0, 255), nSigma=9)
        
        #t0 = time.time()
        image = imutils.resize(image, width=400)
        #print 'Resizing {}'.format(time.time() - t0)
         
        #t0 = time.time()
        #ret, jpeg = cv2.imencode('.jpg', image)
        #print 'encoding {}'.format(time.time() - t0)

        return image #jpeg.tobytes()

if __name__ == '__main__':
    from imutils.video import WebcamVideoStream
    vs0 = WebcamVideoStream(src=0).start()
    #vs0.stream.set(3, 1920)
    #vs0.stream.set(4, 1080)
    
    vs1 = WebcamVideoStream(src=1).start()
    #vs1.stream.set(3, 1920)
    #vs1.stream.set(4, 1080)
 
    vs2 = WebcamVideoStream(src=2).start()
    #vs2.stream.set(3, 1920)
    #vs2.stream.set(4, 1080)
    
    #cameras = VideoCamera()
    
    while (True):
        #image = cameras.get_frame()
        frame0 = vs0.read()
        frame1 = vs1.read()
        frame2 = vs2.read()
        image = computeStitch(frame2, frame1, frame0)
        image = imutils.resize(image, width=400)
        cv2.imshow('frame', image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    del vs0
    del vs1
    del vs2

