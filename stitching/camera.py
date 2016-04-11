import cv2
import numpy as np
from numpy import linalg as LA
import numpy as np
import imutils
import socket
from WarpImages import *

UDP_IP = "35.2.127.133"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

X_SCALE = 0.0014739 
Y_SCALE = 0.00146313
	

def plotcov2d (image, center, covariance, color=(255, 0, 0), nSigma=3):
  w, v = LA.eig(covariance)
  
  covariance /= X_SCALE**2
  
  lam1 = w[0]; lam2 = w[1];
  v1 = v[:,0]; v2 = v[:,1];
  if v1[0] == 0:
    angle = 90
  else:
    angle = -np.math.degrees(np.math.atan(v1[1]/v1[0]));
  
  
  width = nSigma * np.sqrt(lam1) * 1000;
  height = nSigma * np.sqrt(lam2) * 1000;
  
  box = (center, (width, height), angle);
  
  cv2.circle(image, center, 15, color, -1);
  cv2.ellipse(image, box, color, 10)



if __name__ == '__main__':
    from imutils.video import WebcamVideoStream
    vs0 = WebcamVideoStream(src=0).start()
    vs1 = WebcamVideoStream(src=1).start()
    vs2 = WebcamVideoStream(src=2).start()
    
    idx = 0
    groundtruth = []
    while (True):
        frame0 = vs0.read()
        frame1 = vs1.read()
        frame2 = vs2.read()
        image = computeStitch(frame2, frame1, frame0)
        #image = imutils.resize(image, width=400)
        
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        parts_str = data.strip().split()
        parts = map(float, parts_str)
        x, y, th = parts[0:3]
        P = np.array([parts[3:6], parts[6:9], parts[9:12]])
	
	correct = {
	  'idx': idx,
	  'x': x,
	  'y': y,
	  'th': th
	}	
	
	x += 0.46
	y += 0.18
	
        x /= 0.0014739 
        y /= 0.00146313
	
	
	#x = image.shape[0] - x
	y = image.shape[1] - y
	correct['px_x'] = x
	correct['px_y'] = y
	correct['cov'] = P
	
	groundtruth.append(correct)	
        
	filename = 'image_{:05}.png'.format(idx)
	idx += 1
	
	plotcov2d(image, (int(x),int(y)), P)
  	#cv2.circle(image, (int(x), int(y)), 100, (255, 0, 0), -1);
        
        #print "received message:", data
	image = imutils.resize(image, width=800) 	      
	
	cv2.imwrite(filename, image)
        cv2.imshow('frame', image)
	
	

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    import pickle
    ofile = open('results.txt', 'wb')
    pickle.dump(groundtruth, ofile)
    ofile.close()

    del vs0
    del vs1
    del vs2

