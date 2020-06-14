import numpy as np


class Screw:

    @staticmethod
    def x():
        s = np.array([0, 1, 0, 0, 0, 0, 0, 0])
        return s

    @staticmethod
    def y():
        s = np.array([0, 0, 1, 0, 0, 0, 0, 0])
        return s

    @staticmethod
    def z():
        s = np.array([0, 0, 0, 1, 0, 0, 0, 0])
        return s
