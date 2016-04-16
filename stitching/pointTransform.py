import cv2
import numpy as np
import imutils, subprocess

r = 1080; c = 1920
dimR = 6; dimC = 2
middle = [dimR*r/2, dimC*c/2]
robottag = 36


def readFile (filename):
    image = cv2.imread(filename)
    return image


def _saveAndFind (rows, cols, image):
    # Put in image
    finalImage = np.zeros((dimR*r, dimC*c, 3), dtype=np.uint8)
    finalImage[rows[0]:rows[1], cols[0]:cols[1], :] = image
    finalImage = finalImage[250+2100:4860-500, 945:2865, :]

    # Save image
    savename = 'stitched_tmp.png'
    cv2.imwrite(savename, finalImage)
    
    # Run april_detect to find it
    p = subprocess.Popen(['./coordinates', savename], stdout=subprocess.PIPE)
    txt = p.communicate()[0]
    for line in txt.split('\n'):
        if line.strip() == '': continue
        parts = line.strip().split()
        if int(parts[2]) == robottag:
            cols = float(parts[0])
            rows = float(parts[1])
            return [cols, rows]

    return None

def inTop (image):
    H_Top_To_Center = np.array([[0.96815 ,0.016819 ,13.2025 ,],\
                                [0.0076237 ,1.0131 ,280.665 ,],\
                                [-2.4318e-05 ,3.7372e-05 ,1 ,],])
    top    = cv2.warpPerspective(top, H_Top_To_Center, (c,r+r))
    # Indices relating to the top pic
    topR = top.shape[0]
    topC = top.shape[1]
    moveX = 15
    moveY = -1560

    rows = [middle[0]/2-topR/2-moveY, middle[0]/2+topR/2-moveY]
    cols = [middle[1]-topC/2-moveX, middle[1]+topC/2-moveX]

    coords = _saveAndFind(rows, cols, top)
    return coords



def inMiddle (image):
    # Indices relating to the center pic
    rows = [middle[0]-r/2, middle[0]+r/2]
    cols = [middle[1]-c/2, middle[1]+c/2]
    coords = _saveAndFind(rows, cols, image)
    return coords



def inBottom (image):
    H_Bottom_To_Center = np.array([ [0.98035 ,-0.070831 ,-11.6818 ,], \
                                    [-0.0024356 ,0.89546 ,517.6966 ,], \
                                    [9.3846e-06 ,-8.5854e-05 ,1 ,],])
    bottom = cv2.warpPerspective(image, H_Bottom_To_Center, (c,r+r))

    bottomR = bottom.shape[0]
    bottomC = bottom.shape[1]
    moveX = 7
    moveY = 1080

    start = [3*middle[0]/2-bottomR/2-moveY, 3*middle[0]/2+bottomR/2-moveY]
    end = [middle[1]-bottomC/2 - moveX, middle[1]+bottomC/2-moveX]

    coords = _saveAndFind(start, end, bottom)
    return coords
    #coordinates = cv2.warpPerspective(coordinates, H_Top_To_Center, (c,r+r))
    #print coordinates.shape
