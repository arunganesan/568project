# Base Class for each particle (
from utils import minimizedAngle
import numpy as np

class Particle:

    def __init__(self, x = None, dim_x=3, v_std=0, w_std=0, R=0):
        if x is None:
            self.x = zeros((dim_X, 1))
            self.x = x
            self.v_std = v_std
            self.w_std = w_std
            self.R = R

    def sampleOdometery(v, w, dt):
       # Add noise to the control input
       v = np.random.normal(v, v_std)
       w = np.random.normal(w, w_std)
       predict_State(v,w,dt)

    def predict_State(v, w, dt):
        # Deal with zero Gyro reading
        if w == 0: w = 1e-5

        th = x[2]
        x[0] = x[0] + -v/w*math.sin(th) + v/w*math.sin(th + w*dt)
        x[1] = rk.x[1] + v/w*math.cos(th) - v/w * math.cos(th + w*dt)
        x[2] = minimizedAngle(th + w*dt)

    def computeWeight(z, landmarkX, landmarkY):
        dx = landmarkX - x[0]
        dy = landmarkY - x[1]
        h = np.array([minimizedAngle(math.atan2(dy, dx) - x[2])])
        return prob(h-z, self.R)



    def perturb(x_std, y_std, theta_std):
        x[0] = np.random.normal(x[0], x_std)
        x[1] = np.random.normal(x[1], y_std)
        x[2] = np.random.normal(x[2], theta_std)


def prob(a, bsq):
    return  1.0/sqrt(2*math.pi*bsq)*math.exp(-.5*a/bsq)
