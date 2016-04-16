# Base Class for each particle (
import numpy as np
from numpy import eye, array
import math

class Particle:

    def __init__(self, x = array([[0, 0, 0]]).T, dim_x=3, v_std=0, w_std=0, R=0):
        if x is None:
            self.x = np.zeros((dim_x, 1))
        self.x = x
        self.v_std = v_std
        self.w_std = w_std
        self.R = R

    def sampleOdometery(self, v, w, dt):
       # Add noise to the control input
       v = np.random.normal(v, self.v_std, 1)
       w = np.random.normal(w, self.w_std, 1)
       self.predict_State(v,w,dt)

    def predict_State(self, v, w, dt):
        # Deal with zero Gyro reading
        if w == 0: w = 1e-5
        th = self.x[2]
        self.x[0] = self.x[0] + -v/w*math.sin(th) + v/w*math.sin(th + w*dt)
        self.x[1] = self.x[1] + v/w*math.cos(th) - v/w * math.cos(th + w*dt)
        self.x[2] = minimizedAngle(th + w*dt)

    def perturb(self, x_std, y_std, theta_std):
        self.x[0] = np.random.normal(self.x[0], x_std, 1)
        self.x[1] = np.random.normal(self.x[1], y_std, 1)
        self.x[2] = minimizedAngle(np.random.normal(self.x[2], theta_std, 1))

    def computeWeight(self, measurement, landmarkPosition):
        dx = landmarkPosition[0] - self.x[0]
        dy = landmarkPosition[1] - self.x[1]
        h = np.array([minimizedAngle(math.atan2(dy, dx) - self.x[2])])
        print measurement-h
        wt = prob(measurement-h,self.R);
        return wt

# Evaluate probability (for weight computation)
def prob(a, bsq):
    return 1.0/math.sqrt(2.0*math.pi)*math.exp(-.5*a**2 /bsq)

# Find mean/variance of a list of particles (1st moment)
def meanAndVariance(particles):
    NUM_PARTICLES=float(len(particles))

    # Calculate mean
    mean = array([[0.0, 0.0, 0.0]]).T
    cosSum = 0
    sinSum = 0
    for particle in particles:
        mean[0:2] += particle.x[0:2]
        # Special case for theta
        cosSum += math.cos(particle.x[2])
        sinSum += math.sin(particle.x[2])

    mean = mean/NUM_PARTICLES
    mean[2] = math.atan2(sinSum, cosSum)

    # Calculate Variance
    variance = array([[0, 0, 0],[0, 0, 0], [0, 0, 0] ]).T

    for particle in particles:
        shifted = particle.x - mean
        shifted[2] = minimizedAngle(shifted[2])
        variance += shifted*shifted.T
    variance = variance/(NUM_PARTICLES-1)

    return (mean, variance)


def minimizedAngle(theta):
    while theta>math.pi:
        theta -= 2*math.pi

    while theta<-math.pi:
        theta += 2*math.pi

    return theta




