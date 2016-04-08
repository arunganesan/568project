from numpy.random import randn
import math
class RadarSim(object):
    """ Simulates the radar signal returns from an object
    flying at a constant altityude and velocity in 1D.
    """
    def __init__(self, dt, pos, vel, alt):
        self.pos = pos
        self.vel = vel
        self.alt = alt
        self.dt = dt
        def get_range(self):
            """ Returns slant range to the object. Call once
            for each new measurement at dt time from last call.
            """
            # add some process noise to the system
            self.vel = self.vel + .1*randn()
            self.alt = self.alt + .1*randn()
            self.pos = self.pos + self.vel*self.dt
            # add measurement noise
            err = self.pos * 0.05*randn()
            slant_dist = math.sqrt(self.pos**2 + self.alt**2)
            return slant_dist + err
