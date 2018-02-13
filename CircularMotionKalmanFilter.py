# -*- coding: utf-8 -*-
"""
An implementation of the Extended Kalman Filter that specializes in estimating
circular motion variables, including x and y positions, heading direction, and
turning angle and distance traveled between measurements.

The Extended Kalman Filter (EKF) is the nonlinear version of Kalman Filter that
can consider nonlinear dynamic systems. In EKF, the transition and observation
models do not have to be linear, but has to be differentiable. In the case of
circular motion here, the assumption is valid and thus EKF is applicable.
"""

from math import cos, sin
from matrix import matrix

class CircularMotionKalmanFilter(object):
    """
    The CircularMotionKalmanFilter class extended the original KalmanFilter by
    implementing a nonlinear transition model and focusing on using m = 2D
    position measurements to estimate n = 5D variables, including:
        1. x position of the robot
        2. y position of the robot
        3. the robot's current heading direction
        4. the robot's turning angle between each measurements
        5. the distance traveled between each measurements
    Since the five variables to estimate do not form a linear dynamic system,
    the transition matrix F is defined to be the Jacobian of the transition
    model.
    The estimation update step is the same as the original Kalman Filter since
    the observation model H remains linear.
    """
    def __init__(self, cov = 1000.0, measurementNoise = 1.0):
        """ Constructor
        Initializes an Extended Kalman Filter with n=5 variable dimension and
        m=2 measurement dimension, then set up the H and R matrices.
        """
        self.n = 5
        self.m = 2
        self.x = matrix([[]])
        self.x.zero(self.n, 1)
        self.P = matrix([
            [cov, 0, 0, 0, 0],
            [0, cov, 0, 0, 0],
            [0, 0, cov, 0, 0],
            [0, 0, 0, cov, 0],
            [0, 0, 0, 0, cov]
        ])
        self.F = matrix([[]])
        self.F.zero(self.n, self.n)
        self.u = matrix([[]])
        self.u.zero(self.n, 1)
        self.H = matrix([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0]
        ])
        self.R = matrix([
            [measurementNoise, 0],
            [0, measurementNoise]
        ])
        self.I = matrix([[]])
        self.I.identity(self.n)

    def updateMeasurement(self, z):
        """ Update estimations of the EKF based on measurements
        With the input measurements z, updates the x and P matrices.

        Args:
            z: mx1 matrix, the new measurements
        """
        if len(z.value) != self.m or len(z.value[0]) != 1:
            print 'Invalid dimensions for z, should be (%d, 1)' % self.m
            return
        y = z - self.H * self.x
        S = self.H * self.P * self.H.transpose() + self.R
        K = self.P * self.H.transpose() * S.inverse()
        self.x = self.x + K * y
        self.P = (self.I - K * self.H) * self.P

    def predict(self):
        """ Update estimations using Kalman Filter as predictions
        Advnaces in time step and predict the next x and P. Use the current
        estimations to perform the transition model, and use its Jacobian to
        update the covariance matrix P.

        Returns:
            x, y: the predicted x and y positions at the next timestep
        """
        F = self.getTransitionMatrix()
        self.x = self.performTransitionModel()
        self.P = F * self.P * F.transpose()
        return self.x.value[0][0], self.x.value[1][0]

    def performTransitionModel(self):
        """
        Performs the transition model to get the next state of the variables.
        Since we assume a circular motion, x and y positions move by the
        estimated distance along the estimated direction.

        Returns:
            x: nx1 matrix, estimations of the next state
        """
        x = self.getX()
        y = self.getY()
        direction = self.getDirection()
        turnAngle = self.getTurningAngle()
        dist = self.getDistance()
        return matrix([
            [x + dist * cos(direction + turnAngle)],
            [y + dist * sin(direction + turnAngle)],
            [direction + turnAngle],
            [turnAngle],
            [dist]
        ])

    def getTransitionMatrix(self):
        """
        Generate the transition matrix based on the current estimations. Since
        the transition model is nonlinear, the transition matrix would be the
        Jacobian of the transition model. Differentiating the model against the
        variables gives us the matrix as shown below.

        Returns:
            F: nxn matrix, transition matrix / Jacobian of the transition model
        """
        d = self.getDistance()
        angle = self.getDirection() + self.getTurningAngle()
        return matrix([
            [1, 0, -d*sin(angle), -d*sin(angle), cos(angle)],
            [0, 1, d*cos(angle), d*cos(angle), sin(angle)],
            [0, 0, 1, 1, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])

    def predictAfterN(self, n):
        """
        Predicts the state of the variables based on the current estimations
        after n time steps, and return the x and y positions.

        Returns:
            x and y positions estimated to be after n time steps
        """
        x = self.performTransitionModel()
        for i in range(n-1):
            x = self.performTransitionModel()
        return x.value[0][0], x.value[1][0]

    def getX(self):
        """ Get the current estimation of the x position. """
        return self.x.value[0][0]

    def getY(self):
        """ Get the current estimation of the y position. """
        return self.x.value[1][0]

    def getDirection(self):
        """
        Get the current estimation of the heading direction in radians.
        """
        return self.x.value[2][0]

    def getTurningAngle(self):
        """ Get the current estimation of the turning angle. """
        return self.x.value[3][0]

    def getDistance(self):
        """ Get the current estimation of the distance traveled. """
        return self.x.value[4][0]
