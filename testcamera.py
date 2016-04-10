#! /usr/bin/env python
from measure import *
from flow import *

if __name__ == '__main__':
  import time

  m = Measure()
  f = Flow()

  for i in range(30):
    measurements = m.get_measurement()
    motions = m.get_motions()
    print 'T={0}, Measurements={1}, Motions={2}'.format(i, measurements, motions)
    time.sleep(0.25)
  m.kill()
