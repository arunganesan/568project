

import subprocess
import glob

#files = glob.glob('image_*.png')

groundtruth=50
files=glob.glob('images/smaller/image*.png')

#groundtruth = 36
#files = glob.glob('images/larger/image*.png')

groundtruth=36
#files=glob.glob('images/newfullres/fullres*.png')
files=glob.glob('images/newfullres/image*.png')

total_detected = 0
for fname in files:
  p = subprocess.Popen(['./coordinates', fname], stdout=subprocess.PIPE)
  txt = p.communicate()[0]
  detected = None
  for line in txt.split('\n'):
    if line.strip() == '': continue
    parts = line.strip().split()
    if int(parts[2]) == groundtruth:
      detected = line 
      break
  print fname, detected
  if detected: total_detected += 1

print 'Detected {}/{}'.format(total_detected, len(files))
