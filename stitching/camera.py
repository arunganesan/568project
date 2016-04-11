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



if __name__ == '__main__':
    from imutils.video import WebcamVideoStream
    vs0 = WebcamVideoStream(src=0).start()
    vs1 = WebcamVideoStream(src=1).start()
    vs2 = WebcamVideoStream(src=2).start()
    
    while (True):
        frame0 = vs0.read()
        frame1 = vs1.read()
        frame2 = vs2.read()
        image = computeStitch(frame2, frame1, frame0)
        #image = imutils.resize(image, width=400)
        cv2.imshow('frame', image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    del vs0
    del vs1
    del vs2

