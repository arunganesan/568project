import cv2
import numpy as np
import imutils

def computeStitch(bottom, center, top):


    # H21
    H_Top_To_Center = np.array([[0.96815 ,0.016819 ,13.2025 ,],[0.0076237 ,1.0131 ,280.665 ,],[-2.4318e-05 ,3.7372e-05 ,1 ,],])

    H_Bottom_To_Center = np.array([[0.98035 ,-0.070831 ,-11.6818 ,],[-0.0024356 ,0.89546 ,517.6966 ,],[9.3846e-06 ,-8.5854e-05 ,1 ,],])

    def rz(image):
        return imutils.resize(image, width = 800)

    def ds(n,image):
        cv2.imshow(n, image)
        #cv2.waitKey(0)




    """
    # Display All
    ds("top", top)
    ds("center", center)
    ds("bottom", bottom)
    cv2.waitKey(0)
    """

    r = center.shape[0]
    c = center.shape[1]
    #Create final image

    dimR = 6
    dimC = 2
    finalImage = np.zeros((dimR*r, dimC*c, 3), dtype=np.uint8)


    ################################################
    # Warping the top and bottom
    ################################################


    # Warp top
    top    = cv2.warpPerspective(top, H_Top_To_Center, (c,r+r))
    #cv2.imshow("Top", rz(top))    

    # Warp bottom
    bottom = cv2.warpPerspective(bottom, H_Bottom_To_Center, (c,r+r))
    #cv2.imshow("Botton", rz(bottom))



    # Keep middle the same
    #cv2.imshow("center", rz(center))
    #cv2.waitKey(0)


    ################################################
    # Filling in the picture
    ################################################
    middle = [dimR*r/2, dimC*c/2]
    
    # Indices relating to the top pic
    topR = top.shape[0]
    topC = top.shape[1]
    moveX = 15
    moveY = -1560
    startIndexRow_top = middle[0]/2-topR/2-moveY
    endIndexRow_top = middle[0]/2+topR/2-moveY
    startIndexCol_top = middle[1]-topC/2-moveX
    endIndexCol_top = middle[1]+topC/2-moveX  

    # Indices relating to the bottom pic
    bottomR = bottom.shape[0]
    bottomC = bottom.shape[1]
    moveX = 7
    moveY = 1080
    startIndexRow_bottom = 3*middle[0]/2-bottomR/2-moveY
    endIndexRow_bottom = 3*middle[0]/2+bottomR/2-moveY
    startIndexCol_bottom = middle[1]-bottomC/2 - moveX
    endIndexCol_bottom = middle[1]+bottomC/2-moveX   

    # Indices relating to the center pic
    startIndexRow_center = middle[0]-r/2
    endIndexRow_center = middle[0]+r/2
    startIndexCol_center = middle[1]-c/2
    endIndexCol_center = middle[1]+c/2 


    ######################################################
    # Without Mean
    ######################################################
    # Replace top image
    #finalImage[startIndexRow_top:endIndexRow_top, \
    #            startIndexCol_top:endIndexCol_top,:] = top 

    # Replace Bottom Image
    finalImage[startIndexRow_bottom:endIndexRow_bottom, \
                startIndexCol_bottom:endIndexCol_bottom,:] = bottom 

    # Fill in center
    #finalImage[startIndexRow_center:endIndexRow_center, \
    #           startIndexCol_center:endIndexCol_center,:] = center 


    ######################################################
    # With Mean
    ######################################################
    # Replace top image
    #finalImage[startIndexRow_top:endIndexRow_top, \
    #            startIndexCol_top:endIndexCol_top,:] = top
    opaqueTop =  startIndexRow_center - startIndexRow_top

    # Topmost camera 1 part
    finalImage[startIndexRow_top:startIndexRow_center, \
                startIndexCol_top:endIndexCol_top,:] = top[0:opaqueTop, :, :]

    # Top-Middle Overlap part
    t = finalImage[startIndexRow_center:endIndexRow_top, \
                startIndexCol_top:endIndexCol_top,:]
    overlapMiddle = 100
    
    finalImage[startIndexRow_center:startIndexRow_center+overlapMiddle, \
                startIndexCol_top:endIndexCol_top,:] \
                = .5*center[0:overlapMiddle,:,:] \
                   + .5*top[opaqueTop:opaqueTop + overlapMiddle, :, :]

    # Middle opaque part
    lastPart = 100
    middlePart = 880 - lastPart
    finalImage[startIndexRow_center+overlapMiddle:startIndexRow_center+overlapMiddle + middlePart, \
                startIndexCol_top:endIndexCol_top,:] \
                = center[overlapMiddle:overlapMiddle+middlePart,:,:] 


    # Lower Overlap
    fI_R = startIndexRow_center+overlapMiddle + middlePart

    finalImage[fI_R:fI_R+ lastPart, \
                startIndexCol_top:endIndexCol_top,:] \
    = .5*bottom[fI_R - startIndexRow_bottom:fI_R - startIndexRow_bottom + lastPart,:,:] \
      + .5*center[overlapMiddle+middlePart:overlapMiddle+middlePart+lastPart,:,:] 
    

    # Replace Bottom Image
    #finalImage[startIndexRow_bottom:endIndexRow_bottom, \
    #            startIndexCol_bottom:endIndexCol_bottom,:] = bottom 

    # Fill in center
    #finalImage[startIndexRow_center:endIndexRow_center, \
    #           startIndexCol_center:endIndexCol_center,:] = center 



    # Show the final image
    finalImage = finalImage[250+startIndexRow_top:endIndexRow_bottom-500,\
                            startIndexCol_top:endIndexCol_top,:]
    #cv2.imshow("Final Image", rz(finalImage))
    #cv2.waitKey(0)
    return finalImage




if __name__ == '__main__':
    # Read in the image
    bottom = (cv2.imread('door1.jpg'))
    center = (cv2.imread('center1.jpg'))
    top    = (cv2.imread('wall1.jpg'))

    computeStitch(bottom, center, top)
    
