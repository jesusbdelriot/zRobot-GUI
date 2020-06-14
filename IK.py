import time
from Desired import *
from Jacobian import *
from Integral import *
from Parameters import *
from tkinter import *
from tkinter import ttk

D = Desired()
FK = ForwardKinematics()
J = Jacobian()
I = Integral()
P = Parameters()


class InverseKinematics:
    @staticmethod
    def compute(q = np.zeros(P.links().shape[0]), p = np.zeros(3), o = np.zeros(3), a = np.zeros(3), s = 'd', f = 'p'):
        m = q.shape[0]
        n = p.shape[1]
        j = np.zeros((m, n))
        for k in range(0, n):
            d = D.quaternion(p[:, k], o[:, k], a[:, k])
            j[:, k], z = InverseKinematics.algorithm(q, d, s, f)
            j[:, k] = InverseKinematics.convert(j[:, k])
        return j

    @staticmethod
    def algorithm(q, d, s='d', f='p', ta=0):
        k = 0
        z = 0
        while k < 10000:
            k += 1
            c = time.clock()
            t = c-ta
            r, n = FK.classical(q)
            e = d-r
            if abs(e[0]) < 1e-3 and abs(e[1]) < 1e-3 and abs(e[2]) < 1e-3 and abs(e[3]) < 1e-3 and abs(e[4]) < 1e-3 and abs(e[5]) < 1e-3 and abs(e[6]) < 1e-3 and abs(e[7]) < 1e-3:
                z = 1
                q = InverseKinematics.convert(q)
                break
            j = J.quaternion(q)
            if s == 'C' or s == 'c':
                if f == 'P' or f == 'p':
                    w = np.linalg.pinv(j).dot(e)
                elif f == 'T' or f == 't':
                    w = np.transpose(j).dot(e)
                q = I.vector(w, q, t)
            elif s == 'D' or s == 'd':
                if f == 'P' or f == 'p':
                    q = q+(np.linalg.pinv(j).dot(e))
                elif f == 'T' or f == 't':
                    q = q+(np.transpose(j).dot(e))
            ta = c
        return q, z

    @staticmethod
    def convert(q):
        for i in range(q.shape[0]):
            q[i] = ((q[i] / (2 * np.pi)) - int((q[i] / (2 * np.pi)))) * (2 * np.pi)
            if q[i] < -np.pi:
                q[i] = q[i]+2*np.pi
            elif q[i] > np.pi:
                q[i] = q[i]-2*np.pi
        return q

    @staticmethod
    def signal(q, t, dt):
        m, n = q.shape
        for j in range(n):
            if t[j] < dt:
                t[j] = dt
        k = 0
        for j in range(1, n):
            z = int(t[j]/dt)
            k += z
        s = np.zeros((m, k*100))
        for j in range(1, n):
            z = int(t[j]/dt)
            for i in range(z):
                k += 1
                s[:, k] = q[:, j - 1] + ((q[:, j] - q[:, j - 1])*(i/z))
        return s
