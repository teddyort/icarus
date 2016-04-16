#!/usr/bin/env python
import unittest
from controller.Learner import *


class TestLearner(unittest.TestCase):
    def test_learner_accept(self):
        learner = Learner(lb=[0, 0, 0], ub=[0, 0, 0])

        # Always true when fval > fnval
        self.assertTrue(learner.accept(0.75, 0.5, 1))

        # Always false for very small temperature and fval < fnval
        self.assertFalse(learner.accept(0.5, 0.75, 1e-5))

        # Always true for very high temperature
        self.assertTrue(learner.accept(0.5, 0.75, 1e5))

    def test_inlim(self):
        lb = [-1, 0, 1]
        ub = [0, 5, 2]
        learner = Learner(lb, ub)

        # Good point
        self.assertTrue(learner.inlim([0, 1, 1.9], lb, ub))

        # One too low
        self.assertFalse(learner.inlim([0, -1, 1.5], lb, ub))

        # All too high
        self.assertFalse(learner.inlim([1, 6, 1e3], lb, ub))

    def test_no_x0_throws_exception(self):
        learner = Learner(lb=[0, 0, 0], ub=[0, 0, 0])

        err_thrown = False
        try:
            learner.get_next_point()
        except LearnerError:
            err_thrown = True
        self.assertTrue(err_thrown)

    def test_annealing(self):
        # Test the simulated annealing function with a parabaloid objective
        learner = Learner(lb=[-1, -1, -1], ub=[1, 1, 1])
        learner.step = 0.25
        learner.x = [1, 1, 1]
        learner.fval = self.parabaloid(learner.x)
        learner.N = 100

        for i in range(0, 100):
            xn = learner.get_next_point()
            fnval = self.parabaloid(xn)
            learner.add_sample(xn, fnval)

        # We should be very close by now
        np.testing.assert_array_almost_equal(learner.xopt, [0, 0, 0], 1,
                                             "Incorrect minimum returned from simulated annealing.")
        self.assertAlmostEqual(learner.fopt, 0, 1)

    def parabaloid(self, x):
        x = np.array(x)
        return np.dot(x, x)


if __name__ == '__main__':
    import rosunit

    rosunit.unitrun('controller', 'learner', TestLearner)
