from pointTransform import *

import subprocess
import glob

#files = glob.glob('image_*.png')

groundtruth=50
#files=glob.glob('images/smaller/image*.png')

#groundtruth = 36
#files = glob.glob('images/larger/image*.png')

groundtruth=36
#files=glob.glob('images/newfullres/fullres*.png')

# Files were actually mislabelled. The door files are actually wall, i.e. top
files_top =glob.glob('images/frommemory/door*.png')
files_middle = glob.glob('images/frommemory/center*.png')
files_bottom = glob.glob('images/frommemory/wall*.png')

allfiles = zip(files_top, files_middle, files_bottom)

total_detected = 0

for top, middle, bottom in allfiles:

  p = subprocess.Popen(['./coordinates', top], stdout=subprocess.PIPE)
  txt = p.communicate()[0]
  detected_top = None
  for line in txt.split('\n'):
    if line.strip() == '': continue
    parts = line.strip().split()
    if int(parts[2]) == groundtruth:
      print 'Top: ' + line
      detected_top = line
      break

  p = subprocess.Popen(['./coordinates', middle], stdout=subprocess.PIPE)
  txt = p.communicate()[0]
  detected_middle = None
  for line in txt.split('\n'):
    if line.strip() == '': continue
    parts = line.strip().split()
    if int(parts[2]) == groundtruth:
      print 'Middle: ' + line
      detected_middle = line
      break


  p = subprocess.Popen(['./coordinates', bottom], stdout=subprocess.PIPE)
  txt = p.communicate()[0]
  detected_bottom = None
  for line in txt.split('\n'):
    if line.strip() == '': continue
    parts = line.strip().split()
    if int(parts[2]) == groundtruth:
      print 'Bottom: ' + line
      detected_bottom = line
      break


  if detected_top is None and detected_middle is None and detected_bottom is None:
    detected = False
  else:
    detected = True
    total_detected += 1
    if detected_middle:
      coords = inMiddle(readFile(middle))
    elif detected_top:
      coords = inTop(readFile(top))
    elif detected_bottom:
      coords = inBottom(readFile(bottom))




  print bottom, detected, coords
  exit(1)

print 'Detected {}/{}'.format(total_detected, len(allfiles))
