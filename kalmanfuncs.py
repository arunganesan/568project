import numpy as np, math
from utils import minimizedAngle


# Jacobian of the measurement function w.r.t the state. This is used in the
# measurement update.
def HJacobian_at(x, landmarkPosition):
    """ compute Jacobian of H matrix for state x """
    dx = landmarkPosition[0] - x[0]
    dy = landmarkPosition[1] - x[1]
    q  = dx**2 + dy**2

    # For when we add range again!
    # H  =  np.array ([[-dx/sqrt(q), -dy/sqrt(q),  0, ],
    #                  [ dy/q      , -dx/q      , -1, ]])

    H  =  np.array ([[ (dy/q)[0] ,(-dx/q)[0], -1]])
    return H

# Measurement model from Probabilistic Robotics, page 207
# Right now this is only 1 dimensional. Once we add range, we need to make
# it a two-dimensional vector.
def hx(x, landmarkPosition):
    """ Compute Expected measurement """
    dx = landmarkPosition[0] - x[0]
    dy = landmarkPosition[1] - x[1]
    q  = dx**2 + dy**2

    h = np.array([minimizedAngle(math.atan2(dy, dx) - x[2])])
    #print 'Expected = {}'.format(math.degrees(h))

    return h


# Our own prediction function. It updates the state and the covariance.
def predict_State(rk, dt):
    v = rk.u[0]
    w = math.radians(rk.u[1])
    if w == 0: w = 1e-5
    th = rk.x[2]
    
    rk.x[0] = rk.x[0] + -v/w*math.sin(th) + v/w*math.sin(th + w*dt)
    rk.x[1] = rk.x[1] + v/w*math.cos(th) - v/w * math.cos(th + w*dt)
    rk.x[2] = minimizedAngle(th + w*dt)

    rk.P = np.dot(np.dot(rk.F, rk.P), rk.F.T) + rk.Q
    return rk
