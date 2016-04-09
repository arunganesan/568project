#! /usr/bin/env python

import numpy as np

class DetectMotion(picamera.array.PiMotionAnalysis):
    def analyse(self, a):
        a = np.sqrt(
            np.square(a['x'].astype(np.float)) +
            np.square(a['y'].astype(np.float))
            ).clip(0, 255).astype(np.uint8)
        # If there're more than 10 vectors with a magnitude greater
        # than 60, then say we've detected motion
        if (a > 60).sum() > 10:
            #print('Motion detected!')

class Flow:
  def __init__ (self, debug_mode=False):
    import picamera
    from multiprocessing import Pipe, Process
    
    self.debug_mode = debug_mode
    self.parent_conn, self.child_conn = Pipe()
    self.p = Process(target=self.do_motion_detect, args=(self.child_conn,))
    self.p.start()
  
  # Send kill command to child process
  def kill (self):
    self.parent_conn.send('STOP')
    #self.parent_conn.recv()
    self.p.join()
  
  # Checks if pipe has any data
  # If it does return that
  # Else return []
  def get_motion (self):
    if self.parent_conn.poll(0.1) == False:
      return -1
      
    # Else
    motions = self.parent_conn.recv()
    if motions[-1] == 
    return motions
  
  def do_motion_detect (self, child_conn):
    import picamera, subprocess, time
    from image_to_angle import get_angle
    camera = picamera.PiCamera()
    
    while True:
      try:
        camera.resolution = (640, 480)
        camera.start_recording(
           '/dev/null', format='h264', motion_output=output)
        camera.wait_recording(30)
        camera.stop_recording()
        
        if child_conn.poll(0.1):
          s = child_conn.recv()
          if s == 'STOP': break
      
      except KeyboardInterrupt:
          break
    camera.close()
  
if __name__ == '__main__':
  import time
  
  m = Measure()
  for i in range(30):
    measurements = m.get_measurement()
    print 'T={0}, Measurements={1}'.format(i, measurements)
    time.sleep(0.25)
  m.kill()
