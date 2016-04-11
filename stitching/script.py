


"""
1. Iterate through all images
For each image,
  run april tag to get the location of tag # 0
  if tag 0 not visible, continue
  
  
  plot red circle on the tag 0
  plot  trajectory of points up to that point
  also plot trajectory of robot
  save as new file


"""

import glob
directory = 'measurement-2/'
images = glob.glob('{}/*.png'.format(directory))
print sorted(images)

import pickle
locations = pickle.load(open('{}/result.txt'.format(directory)))

import cv2, subprocess
for idx, image_file in enumerate(sorted(images)):
  image = cv2.imread(image_file)
  cmd = './april -d {}'.format(filename).split()
  p = subprocess.
