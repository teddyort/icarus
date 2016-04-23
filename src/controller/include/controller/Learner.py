#!/usr/bin/env python
import numpy as np


class LearnerError(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


class Learner(object):
    def __init__(self, lb, ub, x0=None):
        # Setup default parameters
        self.step = 0.75
        self.T = 1
        self.Tred = 0.9
        self.N = 20
        self.x = x0
        self.fval = 20
        self.xopt = self.x
        self.fopt = self.fval
        self.ws_lb = np.array(lb)
        self.ws_ub = np.array(ub)
        self.ws = self.ws_ub - self.ws_lb
        self.count = 0

    def add_sample(self, xn, fnval):
        if self.accept(self.fval, fnval, self.T):
            # Check if this is the best point so far
            if fnval < self.fopt:
                self.xopt = xn
                self.fopt = fnval
            self.x = xn
            self.fval = fnval

        # Cool down
        self.T *= self.Tred

        # Increment count
        self.count += 1

    def get_next_point(self):
        # Check that x was set
        if self.x is None:
            raise LearnerError("get_next_point() called but initial x was not set.")

        # If we've cooled don't guess
        if self.count > self.N:
            return self.xopt

        # Generate a random valid point in the neighborhood
        while True:
            # Calculate an interval centered on x, with dimensions = step*ws
            box = self.ws * self.step / 2
            lint = np.max(np.vstack((self.x - box, self.ws_lb)), 0)
            uint = np.min(np.vstack((self.x + box, self.ws_ub)), 0)
            x = np.random.uniform(lint[0], uint[0])
            y = np.random.uniform(lint[1], uint[1])
            z = np.random.uniform(lint[2], uint[2])
            if self.inlim([x, y, z], self.ws_lb, self.ws_ub):
                return [x, y, z]
            else:
                print "Couldn't use point %s, generating a new one" % [x, y, z]

    def accept(self, fval, fnval, t):
        return fval > fnval or self.metro(fval, fnval, t)

    @staticmethod
    def metro(fval, fnval, T):
        return np.random.rand() < np.exp((fval - fnval) / T)

    @staticmethod
    def inlim(x, lb, ub):
        return (lb <= np.array(x)).all() and (np.array(x) <= ub).all()
