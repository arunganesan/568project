# np.dot(rk.F, rk.xi Require different functions to run (This seems like bad practice) 
import sys
import os
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
import time
import math
from math import sqrt

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('-x', type=float, default=0)
parser.add_argument('-y', type=float, default=0)
parser.add_argument('-theta', type=float, default=0)
args = parser.parse_args()

# Measurement model from Probabilistic Robotics, page 207
def printStuff(rk, measurements):
    print rk.x[0], rk.x[1], rk.x[2], rk.x[3], rk.x[4], measurements
    #print rk.u[1], rk.u[1]

def HJacobian_at(x, landmarkPosition):
    """ compute Jacobian of H matrix for state x """
   
    dx = landmarkPosition[0] - x[0]
    dy = landmarkPosition[1] - x[1]
    q  = dx**2 + dy**2
    # H  =  np.array ([[-dx/sqrt(q), -dy/sqrt(q),  0, ],
    #                  [ dy/q      , -dx/q      , -1, ]]) 
    
    H  =  np.array ([[ (dy/q)[0]      , (-dx/q)[0]      , -1,  0,  0]]) 
    # print H.shape
    return H

# Measurement model from Probabilistic Robotics, page 207
def hx(x, landmarkPosition):
    """ Compute Expected measurement """
    dx = landmarkPosition[0] - x[0]
    dy = landmarkPosition[1] - x[1]
    q  = dx**2 + dy**2 
    
    rad = math.radians(x[2])
    
    h = np.array([[minimizedAngle(math.atan2(dy, dx) - rad)]])
    return h

def predict_State(rk, dt):
    a_x   = rk.u[0];
    a_y   = rk.u[1];
    omega = rk.u[2];
    
    # Account for zero velocities
    rk.x = np.dot(rk.F, rk.x) + np.array([[.5*a_x*dt*dt, .5*a_y*dt*dt, omega*dt/14.375, a_x*dt, a_y*dt]]).T
    #print a_y, dt
    #rk.x[0] = rk.x[0] + .5*a_x*dt*dt;
    #rk.x[1] = rk.x[1] + .5*a_y*dt*dt;
    rk.x[2] = omega;

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

# Accelerometer/Gyroscope max Update rate = 100 Hz(?)
dt = .01 # TBD

# Initialize Extended Kalman Filter
rk = ExtendedKalmanFilter(dim_x=5, dim_z=1)

# Initialize IMU
imu = IMU()
while True:
    l = imu.get_latest()
    imu.clear_all()
    if l != None: break

# Initialize measurement
measure = Measure(debug_mode=False)

# Make an imperfect sta rting guess
#rk.x = array([0, 0 , 0]) #x, y, theta, v_x, v_y

rk.x = array([[args.x, args.y, args.theta, 0, 0]]).T #x, y, theta, v_x, v_y
"""
rk.F = array([[1, 0, 0],
              [0, 1, 0],
              [0, 0, 1]])
"""

# Noise Parameters (will require tuning)
range_std = .01 # metersi
range_angle = 5 

# Motion Noise
#rk.Q = np.diag([range_std**2, range_std**2, range_std**2 ])

rk.Q = np.diag([range_std**2, range_std**2, math.radians(range_angle)**2, 1, 1])

# Measurement Noise
#rk.R = np.diag([range_std**2, range_std**2])
rk.R = np.array([[math.radians(range_angle)**2]])

# This might imply correlation which in our case is unfounded
#rk.Q[0:2, 0:2] = Q_discrete_white_noise(2, dt=dt, var=0.1)
#rk.Q[2,2] = 0.1

# For some reason, Sigma is called P here....
#rk.P = np.diag([range_std**2, range_std**2, range_std**2 ])
rk.P = np.diag([range_std**2, range_std**2, range_std**2,  range_std**2, range_std**2])

# Test IMU
num = 10
u = imu.get_latest()
for i in range(0,num-1):
    u += imu.get_latest()
    time.sleep(dt)

u = u/num
offsetU = u

#while True:
#    u = imu.getControlInput() - offsetU
#    a = u[0]*.001*9.81
#    b = u[1]*.001*9.81
#    c = u[2]*.001*9.81
#    print a, b, c

# Evolve the state forever 
t1 = time.time()
t2 = 0
xs, track = [], []
i = 1;


try:
    while True:
        # print i
        #z = radar.get_range()
        
        #track.append((radar.pos, radar.vel, radar.alt))

        #rk.update(array([z]), HJacobian_at, hx)
        
        #xs.append(rk.x)
        #rk.predict()
        #xs = asarray(xs)
        #track = asarray(track)
        #time = np.arange(0, len(xs)*dt, dt)

        # Track timestep
        t2 = time.time()
        diff = t2-t1
        t1 = t2
        
        #################################################
        # Prediction Step
        #################################################
        
        # Recieve Control
        #print 'getting state'
        rk.u = imu.get_latest() - offsetU
        #print 'got state'
        imu.clear_all()
        
        
        # Change process matrix accordingly
        rk.F = array([[1, 0, 0, diff,  0],
                  [0, 1, 0,  0, diff],
                  [0, 0, 1,  0, 0],
                  [0, 0, 0,  1, 0],
                  [0, 0, 0,  0, 1]]) 
        
        # Prediction Step (run my own)
        rk = predict_State(rk, diff)

        # Predict with filterpu

        #rk.B = array([[1/2*diff*diff, 0,             0],
        #              [0,             1/2*diff*diff, 0],
        #              [0,             0,             diff]]);

        #rk.predict(u);
        #rk.x[2] = rk.u[2]

        #################################################
        # Update Step
        #################################################

        # Recieve Measurement
        
        # Measurements is a list of measurements
        # Each item in the list is a dictionary 
        # {'bearing': degrees, 'tag': id}
        # If a measurement is not ready, this returns []
        measurements = measure.get_measurement()
        
        # Data Formal:
        # z = [ range; theta; markerID]
        # First two are used in measurement, last one in calculation of h(x) etc. 
        
        for zm in measurements:
            z = np.array([zm['bearing']])
            markerId = zm['id']
            landmarkPosition = get_landmark(markerId)
            
            z[0] = math.radians(z[0])
            #rk.update(z, HJacobian_at, hx, args=landmarkPosition, hx_args=landmarkPosition)
        
        printStuff(rk, measurements)
        #print i, measurements
        i += 1
        #printMeasurement(measurements)
        #################################################
        # Perform Smoothing
        #################################################


        


        # Debugging stuff
        # i = i+1



        time.sleep(dt)

except KeyboardInterrupt, SystemExit:
   print 'Shutting down'
   imu.kill()
   measure.kill()
   time.sleep(1)
   exit(1)
