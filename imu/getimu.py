#! /usr/bin/env python

class Reading:
    def __init__ (self, x,y,z, yaw,p,r, dt):
        self.x = x;
        self.y = y;
        self.z = z;

        self.yaw = yaw;
        self.p = p;
        self.r = r;

        self.dt = dt;


    def get_nparr (self):
        import numpy as np
        return np.array([self.x, self.y, self.z, \
                        self.yaw, self.p, self.r, self.dt])
    
    def __str__ (self):
        return 'dt={6:02} Acc=({0:02},{1:02},{2:02}) Gyro=({3:02},{4:02},{5:02})'.\
                format( self.x, self.y, self.z, \
                        self.yaw, self.p, self.r, self.dt);


    def __repr__ (self):
        return self.__str__()

import numpy as np

class IMU:
  def __init__ (self):

    from multiprocessing import Pipe, Process
    self.readings = []
    self.parent_conn, self.child_conn = Pipe()
    self.p = Process(target=self.read_continuous, args=(self.child_conn,))
    self.p.start()



  # Send kill command to child process
  def kill (self):
    self.parent_conn.send('STOP')
    self.parent_conn.recv()
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
        nparr[0] = self.readings[-1].x
        nparr[1] = self.readings[-1].y
        nparr[2] = self.readings[-1].r
        return nparr

  def get_averaged (self):
      import numpy as np
      arrays = np.zeros((len(self.readings), 7))
      for idx, read in enumerate(self.readings):
          nparr = read.get_nparr()
          arrays[idx,:] = nparr
      return np.mean(arrays, 0)


  def clear_all (self):
      self.readings = []

  def read_continuous (self, child_conn):
      import subprocess
      p = subprocess.Popen(['./imu/cppfunc'], stdout=subprocess.PIPE)
      while True:
        line = p.stdout.readline()
        parts = line.split();
        if len(parts) != 7: continue
        parts = map(float, parts)
        #print 'Acc=({0:02},{1:02},{2:02}) Gyro=({3:02},{4:02},{5:02})'.format(*parts)
        reading = Reading(*parts)
        #self.readings.append(reading)
        #print "(from cont, len is {})".format(len(self.readings))
        
        #print 'Got line {}'.format(len(line))
        child_conn.send(reading)
        if child_conn.poll(0.01):
          s = child_conn.recv()
          if s == 'STOP': break

        if not line: break


if __name__ == '__main__':
  import time

  imu = IMU()
  for i in range(30):
    latest = imu.get_latest()
    #smoothed = imu.get_averaged()
    print 'Latest: T={0}, Measurements={1}'.format(i, latest)
    #print 'Smoothed: T={0}, Measurements={1}'.format(i, smoothed)
    imu.clear_all()

    time.sleep(0.25)
  imu.kill()
