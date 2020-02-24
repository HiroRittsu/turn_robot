from numpy import pi
from numpy import array, eye
from numpy import cos, sin
from numpy import dot, sqrt, arctan2
from numpy.random import randn
from numpy.linalg import inv

from time import sleep

from matplotlib import pyplot as plt
import matplotlib.animation as animation

class EKF():
    def __init__(self, init_x, init_y, init_theta, dt):
        self.x = array([init_x, init_y, init_theta])

        self.u = array([dt*cos(init_theta), dt*sin(init_theta), dt])

        self.A = eye(3)
        self.B = eye(3)
        self.C = array([[1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1]])
        self.P = eye(3)
        self.Q = array([[randn(), 0, 0],
                        [0, randn(), 0],
                        [0, 0, randn()]])
        self.R = array([[randn(), 0, 0],
                        [0, randn(), 0],
                        [0, 0, randn()]])


    def prior_state_estimate(self):
        return dot(self.A, self.x.T) + dot(self.B, self.u.T)

    def prior_error_covariance_matrix(self):
        matrix = dot(self.P, self.A.T)
        return dot(self.A, matrix) + self.Q

    def kalman_gain(self, _P):
        matrix = dot(_P, self.C.T)
        matrix = inv(dot(self.C, matrix) + self.R)
        matrix = dot(self.C.T, matrix)
        return dot(_P, matrix)

    def state_estimate(self, y, x, kalman_gain):
        matrix = y.T - dot(self.C, x.T)
        return x + dot(kalman_gain, matrix)

    def posteriori_error_covariance_matrix(self, kalman_gain, _P):
        matrix = eye(3) - dot(kalman_gain, self.C)
        return dot(matrix, _P)

    def kalman_filter(self, x, y, theta):
        value_of_prior_state_estimate = self.prior_state_estimate()
        value_of_prior_error_covariance_matrix = self.prior_error_covariance_matrix()
        kalman_gain = self.kalman_gain(value_of_prior_error_covariance_matrix)

        value_state_estimate = self.state_estimate(array([x, y, theta]), value_of_prior_state_estimate, kalman_gain)

        value_posteriori_error_covariance_matrix = self.posteriori_error_covariance_matrix(kalman_gain, value_of_prior_error_covariance_matrix)

        est_x = value_state_estimate[0]
        est_y = value_state_estimate[1]
        est_theta = value_state_estimate[2]

        self.x = value_state_estimate
        self.P = value_posteriori_error_covariance_matrix

        return [est_x, est_y, est_theta]
