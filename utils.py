# Measurement model from Probabilistic Robotics, page 207
import math
from Particle import *

def next_batch (data):
  if len(data) == 0: return None, data
  items_arr = []
  item = data.pop(0)
  t = item[0]
  items_arr.append(item)
  while (len(data) != 0 and data[0][0] == t):
    items_arr.append(data.pop(0))

  items = {'time': t}
  for item in items_arr:
    name = item[1]
    values = item[2]
    items[name] = values

  return items, data

def printStuff(rk, measurements, diff):
    #print rk.x[0], rk.x[1], rk.x[2], rk.x[3], rk.x[4], measurements, zerod
    ids = [m['id'] for m in measurements]
    measures = [m['bearing'] for m in measurements]
    fmtstring = 'diff={:7.3f} X={:7.3f}  Y={:7.3f}  TH={:7.3f} => {:7.3f} U={:7.3f}  z={}'
    print fmtstring.format(diff, rk.x[0][0], rk.x[1][0], rk.x[2][0],  math.degrees(rk.x[2]), rk.u[0], ids)


def printMatlab (rk, t2, filename):
    ofile = open(filename, 'a')
    data = [rk.x[0][0], rk.x[1][0], rk.x[2][0], rk.P[0,0], rk.P[0,1], rk.P[0,2],\
            rk.P[1,0], rk.P[1,1], rk.P[1,2], rk.P[2,0], rk.P[2,1], rk.P[2,2], t2]
    outstr =  '\t'.join(['{}'.format(d) for d in data]) + '\n'
    ofile.write(outstr)
    ofile.close()
    return outstr


def minimizedAngle(theta):
    while theta>math.pi:
        theta -= 2*math.pi

    while theta<-math.pi:
        theta += 2*math.pi

    return theta

def printParticles(particles, t2, filename):
    num_particles = len(particles)
    ofile = open(filename, 'a')
    mean, P = meanAndVariance(particles)
    data = [mean.x[0][0], mean.x[1][0], mean.x[2][0], rk.P[0,0], rk.P[0,1], rk.P[0,2],\
            rk.P[1,0], rk.P[1,1], rk.P[1,2], rk.P[2,0], rk.P[2,1], rk.P[2,2], t2]
    outstr =  '\t'.join(['{}'.format(d) for d in data]) + '\n'
    ofile.write(outstr)
    ofile.close()
    return outstr


def get_landmark (idx):
  import numpy as np

  W = 2.00025
  H = 2.40665

  landmarks = {
    0: (W, H-0.288925),
    1: (W, H-0.904875),
    2: (W, H-1.50495),
    3: (W, H-2.1082),

    4: (1.51765, 0),
    5: (0.993775, 0),
    6: (0.41275, 0),

    7: (0, 0.3175),
    8: (0, 0.89535),
    9: (0, 1.50812),
    10: (0, 2.15265),

    11: (W-1.6002, H),
    12: (W-1.11125, H),
    13: (W-0.46355, H)
  }

  return np.array(landmarks[idx])
