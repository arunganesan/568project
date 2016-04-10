#! /usr/bin/env python
from measure import *
from flow import *

if __name__ == '__main__':
  import time

  m = Measure()
  f = Flow()

  for i in range(300):
    measurements = m.get_measurement()
    motions = f.get_motion()
    #print 'T={0}, Measurements={1}, Motions={2}'.format(i, measurements, motions)
    print 'T={0}, Measurements={1}'.format(i, measurements)
    time.sleep(0.25)
  m.kill()
