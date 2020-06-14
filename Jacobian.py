from FK import *
from Hamilton import *
from Screw import *

C = Conjugate()
FK = ForwardKinematics()
H = Hamilton()
P = Parameters()
S = Screw()


class Jacobian:

    @staticmethod
    def quaternion(q=np.zeros(P.links().shape[0])):
        r, n = FK.classical(q)
        j = np.zeros((8, n))
        c = np.array([1, 0, 0, 0, 0, 0, 0, 0])
        for i in range(0, n):
            z = np.array([0, (c[1] * c[3]) + (c[0] * c[2]), (c[2] * c[3]) - (c[0] * c[1]),
                          0.5 * ((c[3] * c[3]) - (c[2] * c[2]) - (c[1] * c[1]) + (c[0] * c[0])), 0,
                          (c[1] * c[7]) + (c[5] * c[3]) + (c[0] * c[6]) + (c[4] * c[2]),
                          (c[2] * c[7]) + (c[6] * c[3]) - (c[0] * c[5]) - (c[4] * c[1]),
                          (c[3] * c[7]) - (c[2] * c[6]) - (c[1] * c[5]) + (c[0] * c[4])])
            j[:, i] = H.left(z).dot(r)
            c, m = FK.classical(q, i+1)
        return j

    @staticmethod
    def inertial(q=np.zeros(P.links().shape[0])):
        n = q.shape[0]
        j = np.zeros((8, n))
        for i in range(0, n):
            r, z = FK.modified(q, i + 1)
            j[:, i] = H.left(r).dot(H.right(C.conjugate(r))).dot(S.z())
        return j

    @staticmethod
    def velocity(q=np.zeros(P.links().shape[0]), p=np.zeros(3)):
        j = np.zeros((8, q.shape[0]))
        r = np.array([[0, 0, 0, 0],
                      [0, 0, -p[2], p[1]],
                      [0, p[2], 0, -p[0]],
                      [0, -p[1], p[0], 0]])
        j[0:4, :] = Jacobian.inertial(q)[0:4, 0:q.shape[0]]
        j[4:8, :] = Jacobian.inertial(q)[4:8, 0:q.shape[0]]-(r.dot(Jacobian.inertial(q)[0:4, 0:q.shape[0]]))
        return j
