#! /usr/bin/env python

import numpy as np

class Joystick:
  def __init__ (self):
    
    from multiprocessing import Pipe, Process
    self.readings = []
    self.parent_conn, self.child_conn = Pipe()
    self.p = Process(target=self.read_continuous, args=(self.child_conn,))
    self.p.start()
    

  # Send kill command to child process
  def kill (self):
    self.parent_conn.send('STOP')
    #self.parent_conn.recv()
    self.p.join()


  def _clear_pipe (self):
    while (self.parent_conn.poll(0.01) != False):
        reading = self.parent_conn.recv()
        self.readings.append(reading)
    
    if len(self.readings) > 10:
        print 'Warning: Cleared pipe, {} elements remaining, maybe trashing.'.format(len(self.readings))
    #print 'Cleared pipe. Readings has {} elements'.format(len(self.readings))


  def get_latest (self):
    self._clear_pipe()
    
    if len(self.readings) == 0: return None
    else: 
        nparr = np.zeros(3)
        nparr[0] = self.readings[-1].y
        nparr[1] = self.readings[-1].r
        nparr[2] = self.readings[-1].dt
        return nparr

  def get_averaged (self):
      import numpy as np
      arrays = np.zeros((len(self.readings), 3))
      for idx, read in enumerate(self.readings):
          nparr = read.get_nparr()
          arrays[idx,:] = nparr
      return np.mean(arrays, 0)


  def clear_all (self):
      self.readings = []

  def read_continuous (self, child_conn):
      import subprocess
      
      cmd = 'sudo python diddyRedJoyEncoder.py'
      p = subprocess.Popen(cmd.split(), stdout=subprocess.PIPE)
      while True:
          try:
                line = p.stdout.readline().strip()
                #if line == '': continue
                reading= float(line)
                
                child_conn.send(reading)
                if child_conn.poll(0.01):
                  s = child_conn.recv()
                  if s == 'STOP': break
          except KeyboardInterrupt:
            break
          
          print 'broke loop'

if __name__ == '__main__':
  import time

  joy = Joystick()
  for i in range(30):
    latest = joy.get_latest()
    #smoothed = imu.get_averaged()
    print 'Latest: T={0}, Measurements={1}'.format(i, latest)
    #print 'Smoothed: T={0}, Measurements={1}'.format(i, smoothed)
    joy.clear_all()

    time.sleep(0.25)
  joy.kill()
