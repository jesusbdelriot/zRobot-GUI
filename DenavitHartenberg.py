from Hamilton import *
from Translation import *
from Rotation import *

R = Rotation()
T = Translation()
H = Hamilton()


class DH:

    @staticmethod
    def classical(q=0, d=0, x=0, a=0):
        z = H.left(H.left(H.left(R.z(q)).dot(T.z(d))).dot(T.x(x))).dot(R.x(a))
        return z

    @staticmethod
    def modified(a=0, x=0, q=0, d=0):
        z = H.left(H.left(H.left(R.x(a)).dot(T.x(x))).dot(R.z(q))).dot(T.z(d))
        return z
