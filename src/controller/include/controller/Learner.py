#!/usr/bin/env python
import numpy as np

class Learner(object):
    def __init__(self):
        # Setup default parameters
        self.step = 0.75
        self.T = 1
        self.Tred = 0.9
        self.N = 20
        self.x = [0,0,0]
        self.fval = 4
        self.xopt = [0,0,0]
        self.fopt = 4
        self.lb = [0, -0.15, -0.05]
        self.ub = [0.26, 0.15, 0.18]

    def add_sample(self, xn, fnval):
        if self.accept(self.fval, fnval, self.T):
            # Check if this is the best point so far
            if fnval < self.fval:
                self.xopt = xn
                self.fopt = fnval
            self.x = xn
            self.fval = fnval

        # Cool down
        self.T = self.T * self.Tred

    def get_next_point(self):
        # Generate a random valid point in the neighborhood
        while True:
            lint = np.array(self.x) - self.step
            uint = np.array(self.x) + self.step
            x = np.random.uniform(lint[0], uint[0])
            y = np.random.uniform(lint[1], uint[1])
            z = np.random.uniform(lint[2], uint[2])
            if self.inlim([x,y,z], self.lb, self.ub):
                return [x,y,z]

    def accept(self, fval, fnval, T):
        return fval > fnval or self.metro(fval, fnval, T)

    def metro(self, fval, fnval, T):
        return np.random.rand() < np.exp((fval-fnval)/T)

    def inlim(self, x, lb, ub):
        return (lb <= np.array(x)).all() and (np.array(x) <= ub).all()