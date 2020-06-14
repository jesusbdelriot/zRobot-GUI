from Jacobian import *

P = Parameters()
J = Jacobian()


class Velocity:

    @staticmethod
    def linear(q=np.zeros(P.links().shape[0]), p=np.zeros(3), w=np.zeros(P.links().shape[0])):
        j = J.velocity(q, p)
        v = j.dot(w)
        return v

    @staticmethod
    def angular(q=np.zeros(P.links().shape[0]), p=np.zeros(3), v=np.zeros(8)):
        j = J.velocity(q, p)
        w = np.linalg.pinv(j).dot(v)
        return w

    @staticmethod
    def algorithm(q, p, v, t):
        w = np.zeros(q.shape)
        z = np.zeros((8, v.shape[1]))
        if t == 'a':
            z[5:8, :] = v[:, :]
            for j in range(1, q.shape[1]):
                w[:, j] = Velocity.angular(q[:, j], p[:, j], z[:, j])
        elif t == 'l':
            for j in range(1, q.shape[1]):
                w[:, j] = Velocity.linear(q[:, j], p[:, j], v[:, j])
        return w
