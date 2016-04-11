# Measurement model from Probabilistic Robotics, page 207
import math

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

def printStuff(rk, measurements):
    #print rk.x[0], rk.x[1], rk.x[2], rk.x[3], rk.x[4], measurements, zerod
    ids = [m['id'] for m in measurements]
    measures = [m['bearing'] for m in measurements]
    fmtstring = 'X={:7.3f}  Y={:7.3f}  TH={:7.3f} => {:7.3f} U={:7.3f}  z={}'
    print fmtstring.format(rk.x[0][0], rk.x[1][0], rk.x[2][0],  math.degrees(rk.x[2]), rk.u[0], ids)


def printMatlab (rk, filename):
    ofile = open(filename, 'a')
    data = [rk.x[0][0], rk.x[1][0], rk.x[2][0], rk.P[0,0], rk.P[0,1], rk.P[0,2],\
            rk.P[1,0], rk.P[1,1], rk.P[1,2], rk.P[2,0], rk.P[2,1], rk.P[2,2]]
    ofile.write('\t'.join(['{}'.format(d) for d in data]) + '\n')
    ofile.close()


def minimizedAngle(theta):
    while theta>math.pi:
        theta -= 2*math.pi

    while theta<-math.pi:
        theta += 2*math.pi

    return theta
