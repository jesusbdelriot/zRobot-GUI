from Conjugate import *
from DenavitHartenberg import *
from Parameters import *

C = Conjugate()
DH = DH()
P = Parameters()


class ForwardKinematics:

    @staticmethod
    def classical(q=np.zeros(P.links().shape[0]), n=P.links().shape[0]):
        h = P.classical(q)
        r = np.array([1, 0, 0, 0, 0, 0, 0, 0])
        for i in range(0, n):
            r = H.left(r).dot(DH.classical(h[i, 0], h[i, 1], h[i, 2], h[i, 3]))
        return r, h.shape[0]

    @staticmethod
    def modified(q=np.zeros(P.links().shape[0]), n=P.links().shape[0]):
        h = P.modified(q)
        r = np.array([1, 0, 0, 0, 0, 0, 0, 0])
        for i in range(0, n):
            r = H.left(r).dot(DH.modified(h[i, 0], h[i, 1], h[i, 2], h[i, 3]))
        return r, h.shape[0]

    @staticmethod
    def homogeneous(q=np.zeros(P.links().shape[0])):
        z, n = ForwardKinematics.classical(q)
        r = np.array([z[0], z[1], z[2], z[3], 0, 0, 0, 0])
        p = 2 * H.left(z).dot(C.conjugate(r))
        return p[5:8]
