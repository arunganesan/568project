

import subprocess
import glob

#files = glob.glob('image_*.png')

groundtruth=50
#files=glob.glob('images/smaller/image*.png')

#groundtruth = 36
#files = glob.glob('images/larger/image*.png')

groundtruth=36
#files=glob.glob('images/newfullres/fullres*.png')
files_door =glob.glob('images/frommemory/door*.png')
files_center = glob.glob('images/frommemory/center*.png')
files_wall = glob.glob('images/frommemory/wall*.png')

allfiles = zip(files_door, files_center, files_wall)

total_detected = 0
for door, center, wall in allfiles:
  
  p = subprocess.Popen(['./coordinates', door], stdout=subprocess.PIPE)
  txt = p.communicate()[0]
  detected_door = None
  for line in txt.split('\n'):
    if line.strip() == '': continue
    parts = line.strip().split()
    if int(parts[2]) == groundtruth:
      detected_door = line 
      break
  
  p = subprocess.Popen(['./coordinates', center], stdout=subprocess.PIPE)
  txt = p.communicate()[0]
  detected_center = None
  for line in txt.split('\n'):
    if line.strip() == '': continue
    parts = line.strip().split()
    if int(parts[2]) == groundtruth:
      detected_center = line 
      break
  
  
  p = subprocess.Popen(['./coordinates', wall], stdout=subprocess.PIPE)
  txt = p.communicate()[0]
  detected_wall = None
  for line in txt.split('\n'):
    if line.strip() == '': continue
    parts = line.strip().split()
    if int(parts[2]) == groundtruth:
      detected_wall = line 
      break
  
  
  if detected_door is None and detected_center is None and detected_wall is None:
    detected = False
  else:
    detected = True
    total_detected += 1
  
  print door, detected
print 'Detected {}/{}'.format(total_detected, len(allfiles))
