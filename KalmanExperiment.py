# np.dot(rk.F, rk.xi Require different functions to run (This seems like bad practice) 
import sys
import os
sys.path.append('/home/pi/Documents/dev/diddyborg-tabletop/sensors/IMU_LSM303')
sys.path.append('/home/pi/Documents/dev/diddyborg-tabletop/sensors/IMU_LSM303/Acc-Mag-LSM303-Python')
sys.path.append('/home/pi/Documents/dev/diddyborg-tabletop/sensors/IMU_LSM303/Gyro-L3GD20-Python')


# Run the EKF
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import ExtendedKalmanFilter
from numpy import eye, array, asarray
import numpy as np
import time
import math
from math import sqrt


# Measurement model from Probabilistic Robotics, page 207
def HJacobian_at(x, landmarkPosition):
    """ compute Jacobian of H matrix for state x """

    dx = landmarkPosition[0] - x[0]
    dy = landmarkPosition[1] - x[1]
    q  = dx**2 + dy**2
    H  =  np.array ([[-dx/sqrt(q), -dy/sqrt(q),  0, 0, 0],
                     [ dy/q      , -dx/q      , -1, 0, 0]]) 
    # print H.shape
    return H

# Measurement model from Probabilistic Robotics, page 207
def hx(x, landmarkPosition):
    """ Compute Expected measurement """
    dx = landmarkPosition[0] - x[0]
    dy = landmarkPosition[1] - x[1]
    q  = dx**2 + dy**2 
    h  = np.array([sqrt(q), minimizedAngle(math.atan2(dy, dx) - x[2])])  
    # print h
    return h

# XXX TODO rk.u now has x/y/z and y/p/r and dt 

def predict_State(rk, dt):
    a_x   = rk.u[0]*.01*9.81;
    a_y   = rk.u[1]*.01*9.81;
    omega = rk.u[2];
    #print rk.F
    #print rk.F.shape
    #print rk.x
    #print rk.x.shape
    #print np.dot(rk.F, rk.x) 
    rk.x = np.dot(rk.F, rk.x) + np.array([1/2*a_x*dt*dt, 1/2*a_y*dt*dt, omega*dt, a_x*dt, a_y*dt])
    rk.P = np.dot(np.dot(rk.F, rk.P), rk.F.T) + rk.Q

    return rk 

def minimizedAngle(theta):
    while theta>math.pi:
        theta -= 2*math.pi
    while theta<math.pi:
        theta += 2*math.pi
    return theta

from imu import *
from measure import *

# Accelerometer/Gyroscope max Update rate = 100 Hz(?)
dt = 1/10.0 # TBD

# Initialize Extended Kalman Filter
rk = ExtendedKalmanFilter(dim_x=5, dim_z=2)

# Initialize IMU
imu = IMU()
while True:
    l = imu.get_latest()
    imu.clear_all()
    if l != None: break

# Initialize measurement
measure = Measure()

# Make an imperfect starting guess
rk.x = array([0, 0 , 0 , 0 , 0]) #x, y, theta, v_x, v_y

rk.F = array([[1, 0, 0, dt,  0],
              [0, 1, 0,  0, dt],
              [0, 0, 1,  0, 0],
              [0, 0, 0,  1, 0],
              [0, 0, 0,  0, 1]])


# Noise Parameters (will require tuning)
range_std = .01 # metersi
# Motion Noise
rk.Q = np.diag([range_std**2, range_std**2, range_std**2, range_std**2, range_std**2 ])
# Measurement Noise
rk.R = np.diag([range_std**2, range_std**2])

# This might imply correlation which in our case is unfounded
#rk.Q[0:2, 0:2] = Q_discrete_white_noise(2, dt=dt, var=0.1)
#rk.Q[2,2] = 0.1

# For some reason, Sigma is called P here....
rk.P = np.diag([range_std**2, range_std**2, range_std**2, range_std**2, range_std**2 ])


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
    # t1 = t2
    
    #################################################
    # Prediction Step
    #################################################
    
    # Recieve Control
    rk.u = imu.get_latest()
    imu.clear_all()
    
    # Prediction Step (run my own)
    rk = predict_State(rk, dt)

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
    z = np.array([5, .32, 1])

    # Recieve landmark position
    landmarkPosition = np.array([5,10])

    # Perform Update
    # rk.update(z[0:2], HJacobian_at, hx, args=landmarkPosition, hx_args=landmarkPosition)
    
    print rk.x[4]

    #################################################
    # Perform Smoothing
    #################################################






    # Debugging stuff
    # i = i+1



    time.sleep(dt)

imu.kill()
measure.kill()
