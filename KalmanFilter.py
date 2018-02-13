# -*- coding: utf-8 -*-
"""
A simple implementation of Kalman Filter for Udacity's AI for Robotics final
project that depends on the provided matrix.py.
"""

from matrix import matrix

class KalmanFilter(object):
    """
    The KalmanFilter class uses the conventional notation as described in
    Wikipedia. To estimate variables of n dimensions with measurements of
    m dimensions, It keeps track of:
        x: nx1 matrix, best guess of the variables
        P: nxn matrix, diagonal matrix with the covariances of the estimates
        F: nxn matrix, transition matrix that describes relations between
            variables at consecutive time steps
        u: nx1 matrix, external motion that is usually assumed to be zero
        H: mxn matrix, the relationship between measurement and variables
        R: mxm matrix, diagonal matrix with the measurement noises
        I: nxn matrix, identity matrix
    Whenever it receives a measurement z with dimensions mx1, it performs the
    update:
        y = z - H * x
        S = H * P * H^T + R
        K = P * H^T * S^-1
        x' = x + K * y
        P' = (I - K * H) * P
    To predict the x and P at the next timestep, it uses these steps:
        x' = F * x + u
        P' = F * P * F^T
    """
    def __init__(self, n=1, m=1):
        """ Constructor
        Stores the input dimensions for future error checking, and also
        declares the matrices to empty based on the dimensions.

        Args:
            n: int, the dimension of the variables to be estimated
            m: int, the dimension of the measurements
        """
        self.n = n
        self.m = m
        self.x = matrix([[]])
        self.x.zero(n, 1)
        self.P = matrix([[]])
        self.P.zero(n, n)
        self.F = matrix([[]])
        self.F.zero(n, n)
        self.u = matrix([[]])
        self.u.zero(n, 1)
        self.H = matrix([[]])
        self.H.zero(m, n)
        self.R = matrix([[]])
        self.R.zero(m, m)
        self.I = matrix([[]])
        self.I.identity(n)

    def updateMeasurement(self, z):
        """ Update estimations using Kalman Filter based on measurements
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
        Advnaces in time step and predict the next x and P.

        Returns:
            x: nx1 matrix, current estimations of the variables
            P: nxn matrix, current covariances for the estimations
        """
        self.x = self.F * self.x + self.u
        self.P = self.F * self.P * self.F.transpose()
        return self.x, self.P
