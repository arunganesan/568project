# np.dot(rk.F, rk.xi Require different functions to run (This seems like bad practice) 
import sys
import os, pickle
sys.path.append('/home/pi/Documents/dev/diddyborg-tabletop/sensors/IMU_LSM303')
sys.path.append('/home/pi/Documents/dev/diddyborg-tabletop/sensors/IMU_LSM303/Acc-Mag-LSM303-Python')
sys.path.append('/home/pi/Documents/dev/diddyborg-tabletop/sensors/IMU_LSM303/Gyro-L3GD20-Python')


# Run the EKF
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import ExtendedKalmanFilter

# Use below if you want to debug the EKF filter code.
# But actually, probably that code is working well
#from EKF import ExtendedKalmanFilter

from numpy import eye, array, asarray
import numpy as np
import time, math, subprocess
from math import sqrt

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('-x', type=float, default=0.889)
parser.add_argument('-y', type=float, default=0.8509)
parser.add_argument('--theta', type=float, default=0)
parser.add_argument('--debug', action='store_true')
parser.add_argument('--data', type=str)
parser.add_argument('--negativegyro', action='store_true')
args = parser.parse_args()

print args
exit(1)
if 'data' in args:
  print 'Data not yet supported' 

PREDFILE = 'runs/onlypred.txt'
PREDUPDA = 'runs/predupdate.txt'
open(PREDFILE, 'w').close()
open(PREDUPDA, 'w').close()

# Measurement model from Probabilistic Robotics, page 207
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

def HJacobian_at(x, landmarkPosition):
    """ compute Jacobian of H matrix for state x """
    
    dx = landmarkPosition[0] - x[0]
    dy = landmarkPosition[1] - x[1]
    q  = dx**2 + dy**2
    
    # For when we add range again!
    # H  =  np.array ([[-dx/sqrt(q), -dy/sqrt(q),  0, ],
    #                  [ dy/q      , -dx/q      , -1, ]]) 
    
    H  =  np.array ([[ (dy/q)[0]      , (-dx/q)[0]      , -1]]) 
    return H

# Measurement model from Probabilistic Robotics, page 207
def hx(x, landmarkPosition):
    """ Compute Expected measurement """
    dx = landmarkPosition[0] - x[0]
    dy = landmarkPosition[1] - x[1]
    q  = dx**2 + dy**2 
       
    
    h = np.array([minimizedAngle(math.atan2(dy, dx) - x[2])])
    #print 'Expected = {}'.format(math.degrees(h))
    
    return h

def predict_State(rk, dt):
    
    # Account for zero velocities
    #rk.x = np.dot(rk.F, rk.x) + np.array([[.5*a_x*dt*dt, .5*a_y*dt*dt, omega*dt/14.375, a_x*dt, a_y*dt]]).T
    v = rk.u[0]
    w = math.radians(rk.u[1])
    #th = math.radians(rk.x[2])
    th = rk.x[2]
    
    rk.x[0] = rk.x[0] + -v/w*math.sin(th) + v/w*math.sin(th + w*dt)
    rk.x[1] = rk.x[1] + v/w*math.cos(th) - v/w * math.cos(th + w*dt)
    rk.x[2] = minimizedAngle(th + w*dt)
    
    #print a_y, dt
    #rk.x[0] = rk.x[0] + .5*a_x*dt*dt;
    #rk.x[1] = rk.x[1] + .5*a_y*dt*dt;
    #rk.x[2] = omega;
    
    rk.P = np.dot(np.dot(rk.F, rk.P), rk.F.T) + rk.Q
    return rk

def minimizedAngle(theta):
    while theta>math.pi:
        theta -= 2*math.pi
    
    while theta<-math.pi:
        theta += 2*math.pi
    
    return theta

from imu import *
from measure import *
from flow import *
from joystick import *

# Accelerometer/Gyroscope max Update rate = 100 Hz(?)
dt = 0.01 # TBD

# Initialize Extended Kalman Filter
rk = ExtendedKalmanFilter(dim_x=3, dim_z=1)
rk2 = ExtendedKalmanFilter(dim_x=3, dim_z=1)

# Set up joystick
joystick = Joystick()

# Initialize IMU
imu = IMU()
while True:
    l = imu.get_latest()
    imu.clear_all()
    if l != None: break

omega_Y = 0

# Initialize measurement
measure = Measure(debug_mode=False)

# Initialize flow detection
flow = Flow()

# Make an imperfect sta rting guess
#rk.x = array([0, 0 , 0]) #x, y, theta, v_x, v_y

rk.x = array([[args.x, args.y, args.theta]]).T #x, y, theta, v_x, v_y
rk2.x = array([[args.x, args.y, args.theta]]).T #x, y, theta, v_x, v_y
"""
rk.F = array([[1, 0, 0],
              [0, 1, 0],
              [0, 0, 1]])
"""

# Noise Parameters (will require tuning)
range_std = 0.005 # metersi
range_angle = 1
april_angle = 1



# Motion Noise
#rk.Q = np.diag([range_std**2, range_std**2, range_std**2 ])

rk.Q = np.diag([range_std**2, range_std**2, math.radians(range_angle)**2])
rk2.Q = np.diag([range_std**2, range_std**2, math.radians(range_angle)**2])

# Measurement Noise
#rk.R = np.diag([range_std**2, range_std**2])
rk.R = np.array([[math.radians(april_angle)**2]])
rk2.R = np.array([[math.radians(april_angle)**2]])

# This might imply correlation which in our case is unfounded
#rk.Q[0:2, 0:2] = Q_discrete_white_noise(2, dt=dt, var=0.1)
#rk.Q[2,2] = 0.1

# For some reason, Sigma is called P here....
#rk.P = np.diag([range_std**2, range_std**2, math.radians(range_angle)**2])
rk.P = np.diag([0.01**2, 0.01**2, math.radians(1)**2])
rk2.P = np.diag([0.01**2, 0.01**2, math.radians(1)**2])

# Flow related constants
MOTION_THRESH = 10  # The number of pixels that have high motion must be at 
                    # least this to be considered a "motion"

LAST_N = 3 #5         # The last number of frames with no motion has to exceed 
                    # this value before we consider it to be "stopped". If the 
                    # framerate is low, this value needs to be lower. 

VEL_DECAY = 0# 0.75    # Reduce the velocity by this factor if we detect no motion 
                    # in the optical flow
motions = []

# Get offset of IMU
num = 10
u = imu.get_latest()
for i in range(0,num-1):
    u += imu.get_latest()
    time.sleep(dt)

u = u/num
offsetU = u

# Evolve the state forever 
t1 = time.time()
t2 = 0
xs, track = [], []
i = 1;

datadump = []

sys.stderr.write('Started!\n')
try:
    while True:
        # Track timestep
        t2 = time.time()
        diff = t2-t1
        t1 = t2
        
        
        #################################################
        # Flow Update
        #################################################
        
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
        
        motion = imu.get_latest() - offsetU
        datadump.append([t2, 'imu', latest_motion])
        imu.clear_all()
        
        if args.negativegyro: 
            motion[1] = -1*motion[1]
        rk.u = motion
        rk2.u = motion
        """
        velocity_Y += diff*rk.u[0]
        
        # XXX This is not used.
        # We are getting velocity frmo joystick
        rk.u[0] = velocity_Y
        rk2.u[0] = velocity_Y
        """
        # Receiving joystick control
        joy = joystick.get_latest()
        datadump.append([t2, 'joystick', joy])
        joystick.clear_all()
        rk.u[0] = joy
        rk2.u[0] = joy
        
        #print joy 
        # Change process matrix accordingly
        v = rk.u[0]
        w = math.radians(float(rk.u[1]))
        th = rk.x[2]
        
        rk.F = array(   [[1, 0, v/w*(-math.cos(th)+math.cos(th + w*diff))],
                         [0, 1, v/w*(-math.sin(th) + math.sin(th + w*diff))],
                         [0, 0, 1]])
        
        th = rk2.x[2]
        
        rk2.F = array(   [[1, 0, v/w*(-math.cos(th)+math.cos(th + w*diff))],
                         [0, 1, v/w*(-math.sin(th) + math.sin(th + w*diff))],
                         [0, 0, 1]])
        
        
        """
        rk.F = array([[1, 0, 0, diff,  0],
                  [0, 1, 0,  0, diff],
                  [0, 0, 1,  0, 0],
                  [0, 0, 0,  1, 0],
                  [0, 0, 0,  0, 1]]) 
        """
        ## Prediction Step (run my own)
        rk = predict_State(rk, diff)
        rk2 = predict_State(rk2, diff)
        

        #################################################
        # Update Step
        #################################################

        # Recieve Measurement
        
        # Measurements is a list of measurements
        # Each item in the list is a dictionary 
        # {'bearing': degrees, 'tag': id}
        # If a measurement is not ready, this returns []
        measurements =  measure.get_measurement()
        if len(measurements) != 0:
          datadump.append([t2, 'measurements', measurements])
        
        for zm in measurements:
            z = np.array([zm['bearing']])
            markerId = zm['id']
            landmarkPosition = get_landmark(markerId)
            
            z[0] = math.radians(z[0])
            rk.update(z, HJacobian_at, hx, args=landmarkPosition, hx_args=landmarkPosition)
        
        
        
        #print velocity_Y
        #if args.debug:
        printStuff(rk2, measurements)
        #else:
            #sys.stderr.write('Angle: {}\n'.format(math.degrees(rk.x[2])))
        printMatlab(rk, PREDUPDA)
        printMatlab(rk2, PREDFILE)
        #print i, measurements
        i += 1
        
        
        time.sleep(dt)

except KeyboardInterrupt, SystemExit:
   sys.stderr.write( 'Shutting down')
   imu.kill()
   flow.kill()
   measure.kill()
   joystick.kill()
   
   print 'Saving data file'
   ofile = open('data.pkl', 'wb')
   pickle.dump(dump, ofile)
   ofile.close()
   
   time.sleep(1)
   exit(1)
