# np.dot(rk.F, rk.xi Require different functions to run (This seems like bad practice)
import sys
import os, pickle
sys.path.append('/home/pi/Documents/dev/diddyborg-tabletop/sensors/IMU_LSM303')
sys.path.append('/home/pi/Documents/dev/diddyborg-tabletop/sensors/IMU_LSM303/Acc-Mag-LSM303-Python')
sys.path.append('/home/pi/Documents/dev/diddyborg-tabletop/sensors/IMU_LSM303/Gyro-L3GD20-Python')

from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import ExtendedKalmanFilter
from numpy import eye, array, asarray
import numpy as np, time, math, subprocess

from utils import *
from kalmanfuncs import *

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('-x', type=float, default=0.889)
parser.add_argument('-y', type=float, default=0.8509)
parser.add_argument('--theta', type=float, default=0)
parser.add_argument('--debug', action='store_true')
parser.add_argument('--usedata', type=str)
parser.add_argument('--negativegyro', action='store_true')

parser.add_argument('--savefilter', type=str, default='runs/output.txt')
parser.add_argument('--savedata', type=str, default='runs/data.pkl')
args = parser.parse_args()


# Nulling out the files
open(args.savefilter, 'w').close()




#####################################
# CONSTANTS
#####################################

# The sleep time in between Kalman iterations
dt = 0.01
omega_Y = 0

# Initial covariance
XCOV = 0.01
YCOV = 0.01
THCOV = 1

# Noise Parameters (will require tuning)
range_std = 0.005 # metersi
range_angle = 1
april_angle = 1



# Flow related constants
MOTION_THRESH = 10  # The number of pixels that have high motion must be at
                    # least this to be considered a "motion"

LAST_N = 3          # The last number of frames with no motion has to exceed
                    # this value before we consider it to be "stopped". If the
                    # framerate is low, this value needs to be lower.

VEL_DECAY = 0    # Reduce the velocity by this factor if we detect no motion
                    # in the optical flow
















#####################################
# INITIALIZATION
#####################################
# Initializesensors
if args.usedata == None:
  from imu import *
  from measure import *
  from flow import *
  from joystick import *


  # Set up joystick
  joystick = Joystick()

  # Initialize IMU
  imu = IMU()
  while True:
      l = imu.get_latest()
      imu.clear_all()
      if l != None: break
  
  # Get offset of IMU
  num = 10
  u = imu.get_latest()
  for i in range(0,num-1):
      u += imu.get_latest()
      time.sleep(dt)
  u = u/num
  offsetU = u



  # Initialize measurement
  measure = Measure(debug_mode=False)

  # Initialize flow detection
  flow = Flow()


# Initialize Kalman

# Initialize Extended Kalman Filter
rk = ExtendedKalmanFilter(dim_x=3, dim_z=1)

# Starting guess location
rk.x = array([[args.x, args.y, args.theta]]).T

# Motion Noise
rk.Q = np.diag([range_std**2, range_std**2, math.radians(range_angle)**2])

# Measurement Noise
rk.R = np.array([[math.radians(april_angle)**2]])

# Covariance matrix
rk.P = np.diag([XCOV**2, YCOV**2, math.radians(THCOV)**2])










#####################################
# FILTER LOOP
#####################################

# Evolve the state forever
t1 = time.time(); t2 = 0
xs, track = [], []
datadump = []
motions = []

sys.stderr.write('Started!\n')

if args.usedata != None:
  ifile = open(args.usedata, 'r')
  data = pickle.load(ifile) 
  ifile.close()
  
  first_batch, data = next_batch(data)
  t1 = first_batch['time'] # Time of the first element

try:
    while True:
        if args.usedata:
          batch, data = next_batch(data)
          if batch == None: break
        
        # Track timestep
        if args.usedata: t2 = batch['time']
        else: t2 = time.time()

        diff = t2 - t1
        t1 = t2
        

        #################################################
        # Flow Update
        #################################################
        
        if args.usedata: latest_motion = batch['flow']
        else: 
          latest_motion = flow.get_motion()
          datadump.append([t2,'flow', latest_motion])
        
        """
        zerod = False
        if latest_motion != None:
            motions += latest_motion
            motions[:-LAST_N] = []
            if all([m < MOTION_THRESH for m in motions]):
                # Reducing velocities
                velocity_Y = velocity_Y * VEL_DECAY
                zerod = True
        """

        #################################################
        # Prediction Step
        #################################################

        # Recieve Control
        
        if args.usedata: motion = batch['imu']
        else: 
          motion = imu.get_latest() - offsetU
          datadump.append([t2, 'imu', motion])
          imu.clear_all()
        
        if args.negativegyro:
            motion[1] = -1*motion[1]
        rk.u = motion

        """
        velocity_Y += diff*rk.u[0]

        # XXX This is not used.
        # We are getting velocity frmo joystick
        rk.u[0] = velocity_Y
        """
        # Receiving joystick control
        if args.usedata: joy = batch['joystick']
        else:
          joy = joystick.get_latest()
          datadump.append([t2, 'joystick', joy])
          joystick.clear_all()
        
        rk.u[0] = joy

        #print joy
        # Change process matrix accordingly
        v = rk.u[0]
        w = math.radians(float(rk.u[1]))
        th = rk.x[2]

        rk.F = array(   [[1, 0, v/w*(-math.cos(th)+math.cos(th + w*diff))],
                         [0, 1, v/w*(-math.sin(th) + math.sin(th + w*diff))],
                         [0, 0, 1]])




        ## Prediction Step (run my own)
        rk = predict_State(rk, diff)


        #################################################
        # Update Step
        #################################################

        # Measurements is a list of measurements
        # Each item in the list is a dictionary
        # {'bearing': degrees, 'tag': id}
        # If a measurement is not ready, this returns []
        
        if args.usedata: measurements = batch['measurements']
        else:
          measurements =  measure.get_measurement()
          datadump.append([t2, 'measurements', measurements])
        
        for zm in measurements:
            z = np.array([zm['bearing']])
            markerId = zm['id']
            landmarkPosition = get_landmark(markerId)

            z[0] = math.radians(z[0])
            rk.update(z, HJacobian_at, hx, args=landmarkPosition, hx_args=landmarkPosition)



        # Printing state of filter
        printStuff(rk, measurements)
        printMatlab(rk, args.savefilter)


        time.sleep(dt)

except KeyboardInterrupt, SystemExit:
   sys.stderr.write( 'Shutting down')
   imu.kill()
   flow.kill()
   measure.kill()
   joystick.kill()

   print 'Saving data file'
   ofile = open(args.savedata, 'wb')
   pickle.dump(datadump, ofile)
   ofile.close()

   time.sleep(1)
   exit(1)
