#! /usr/bin/env python
from imu import *

if __name__ == '__main__':
  import time
 
  imu = IMU()
  for i in range(300):
    latest = imu.get_latest()
    #smoothed = imu.get_averaged()
    print 'Latest: T={0}, Measurements={1}'.format(i, latest)
    #print 'Smoothed: T={0}, Measurements={1}'.format(i, smoothed)
    imu.clear_all()

    time.sleep(0.05)
  imu.kill()
