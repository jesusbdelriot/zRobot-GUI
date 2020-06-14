from Hamilton import *
from Rotation import *
from Translation import *

H = Hamilton()
R = Rotation()
T = Translation()


class Desired:

    @staticmethod
    def quaternion(p, o, a):
        z = np.array([1, 0, 0, 0, 0, 0.5*p[0], 0.5*p[1], 0.5*p[2]])
        if a == 'X' or a == 'x' or a == 88 or a == 120:
            z = H.left(z).dot(R.x(o))
        elif a == 'Y' or a == 'y' or a == 89 or a == 121:
            z = H.left(z).dot(R.y(o))
        elif a == 'Z' or a == 'z' or a == 90 or a == 122:
            z = H.left(z).dot(R.z(o))
        return z
