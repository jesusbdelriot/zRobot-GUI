import numpy as np


class Translation:

    @staticmethod
    def x(t=0):
        v = np.array([1, 0, 0, 0, 0, 0.5*t, 0, 0])
        return v

    @staticmethod
    def y(t=0):
        v = np.array([1, 0, 0, 0, 0, 0, 0.5*t, 0])
        return v

    @staticmethod
    def z(t=0):
        v = np.array([1, 0, 0, 0, 0, 0, 0, 0.5*t])
        return v
