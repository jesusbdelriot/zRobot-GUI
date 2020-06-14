import numpy as np


class Rotation:

    @staticmethod
    def x(a=0):
        v = np.array([np.cos(0.5 * a), np.sin(0.5 * a), 0, 0, 0, 0, 0, 0])
        return v

    @staticmethod
    def y(a=0):
        v = np.array([np.cos(0.5*a), 0, np.sin(0.5*a), 0, 0, 0, 0, 0])
        return v

    @staticmethod
    def z(a=0):
        v = np.array([np.cos(0.5*a), 0, 0, np.sin(0.5*a), 0, 0, 0, 0])
        return v
