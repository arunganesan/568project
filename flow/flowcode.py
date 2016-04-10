#! /usr/bin/env python

import numpy as np
import picamera
import picamera.array

class DetectMotion(picamera.array.PiMotionAnalysis):
    def set_conn (self, conn):
            self.conn = conn
    def analyse(self, a):
        a = np.sqrt(
            np.square(a['x'].astype(np.float)) +
            np.square(a['y'].astype(np.float))
            ).clip(0, 255).astype(np.uint8)
        # If there're more than 10 vectors with a magnitude greater
        # than 60, then say we've detected motion
        #howmany = (a>60).sum()
        howmany = (a>25).sum()
        self.conn.send(howmany)

class Flow:
  def __init__ (self, debug_mode=False):
    from multiprocessing import Pipe, Process
    self.motions = []
    self.debug_mode = debug_mode
    self.parent_conn, self.child_conn = Pipe()
    self.p = Process(target=self.do_motion_detect, args=(self.child_conn,))
    self.p.start()
  
  # Send kill command to child process
  def kill (self):
    self.parent_conn.send('STOP')
    #self.parent_conn.recv()
    self.p.join()
  
  def _clear_pipe (self):
    while (self.parent_conn.poll(0.01) != False):
        reading = self.parent_conn.recv()
        self.motions.append(reading)
     

  # Checks if pipe has any data
  # If it does return that
  # Else return []
  def get_motion (self):
    self._clear_pipe()
    if len(self.motions) == 0: return None
    else:
        copy = self.motions
        self.motions = []
        return copy
   
  def do_motion_detect (self, child_conn):
    import picamera, subprocess, time
    camera = picamera.PiCamera(framerate=30)
    
    output = DetectMotion(camera)
    output.set_conn(child_conn)
    while True:
      try:
        camera.resolution = (640, 480)
        camera.start_recording(
           '/dev/null', format='h264', motion_output=output)
        camera.wait_recording(5)
        camera.stop_recording()
        
        if child_conn.poll(0.01):
          s = child_conn.recv()
          if s == 'STOP': break
      
      except KeyboardInterrupt:
          break
    camera.close()
  
if __name__ == '__main__':
  import time
  
  m = Flow()
  for i in range(300):
    measurements = m.get_motion()
    print 'T={0}, Flow={1}'.format(i, measurements)
    time.sleep(0.1)
  m.kill()
