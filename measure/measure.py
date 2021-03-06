#! /usr/bin/env python
import os

class Measure:
  def __init__ (self, debug_mode=False):
    from multiprocessing import Pipe, Process
    self.measurements = []
    self.debug_mode = debug_mode
    self.parent_conn, self.child_conn = Pipe()
    self.p = Process(target=self.do_measure, args=(self.child_conn,))
    self.p.start()
  
  # Send kill command to child process
  def kill (self):
    self.parent_conn.send('STOP')
    #self.parent_conn.recv()
    self.p.join()
  
  # Checks if pipe has any data
  # If it does return that
  # Else return []
  def get_measurement (self):
    if self.parent_conn.poll(0.1) == False:
      return []
    
    # Else
    measures = self.parent_conn.recv()
    self.measurements += measures
    returnable = self.measurements
    self.measurements = []
    return returnable
  
  def do_measure (self, child_conn):
    import picamera, subprocess, time
    from image_to_angle import get_angle
    #camera = picamera.PiCamera()
    
    prev_time = 0
    timedelta = 0.05
    while (True):
      try:
          measurements = []
          
          # 1. Get camera image 
          # The flowcode will take pictures for us
          #if self.debug_mode:
          #    filename = 'image-{}.jpg'.format(int(time.time()))
          #else:
          #    filename = 'image.jpg'
          #camera.capture(filename)
          filename = 'image.jpg'
          if not os.path.exists(filename): continue
          try:
            mtime = os.path.getmtime(filename)
          except:
            # Got some error
            continue
        
          if  mtime - prev_time < timedelta: continue
          else: prev_time = mtime
          
          # 2. Run april tags
          cmd = './measure/april -d {}'.format(filename).split()
          p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
          output = p.communicate()[0]
          for line in output.split('\n'):
            parts = line.strip().split()
            if len(parts) == 0: continue
            tag_id = int(parts[0])
            pixel_loc = float(parts[1])
            
            # 3. Use image to angle
            angle = get_angle(100, pixel_loc)
            measurements.append({
              'id': tag_id,
              'bearing': angle[0],
              'time': mtime
            })
          
          child_conn.send(measurements)
          if child_conn.poll(0.1):
            s = child_conn.recv()
            if s == 'STOP': break
      except KeyboardInterrupt:
          break
    #camera.close()
  
if __name__ == '__main__':
  import time
  
  m = Measure()
  for i in range(30):
    measurements = m.get_measurement()
    print 'T={0}, Measurements={1}'.format(i, measurements)
    time.sleep(0.25)
  m.kill()
