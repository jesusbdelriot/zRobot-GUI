from Derivatives import *
from Desired import *
from Velocity import *

D = Derivatives()
Z = Desired()
FK = ForwardKinematics()
P = Parameters()
V = Velocity()


class Paths:

    @staticmethod
    def time(q=np.zeros(P.links().shape[0]), t=1, dt=1):
        if 0 < ((t/dt)-int(t/dt)) < 1 or dt < 1:
            p = np.zeros((3, int(t/dt)+2))
            o = np.zeros((1, int(t/dt)+2))
            a = np.zeros((1, int(t/dt)+2))
        else:
            p = np.zeros((3, int(t/dt)+1))
            o = np.zeros((1, int(t/dt)+1))
            a = np.zeros((1, int(t/dt)+1))
        p[:, 0] = FK.homogeneous(q)
        i = 0
        k = 0
        while i < t:
            k += 1
            p[:, k] = np.array([0.15+(((2*np.pi/t)*i)/20), 0.15+(0.25*np.cos((2*np.pi/t)*i)), 0.15+(0.25*np.sin((2*np.pi/t)*i))])
            o[:, k] = np.pi
            '''x axis = 120 ASCII'''
            a[:, k] = 120
            i = i + dt
        v = D.time(p, dt)
        return p, o, a, v

    @staticmethod
    def spatial(q=np.zeros(P.links().shape[0]), l=0.5, dl=0.001, z = np.array([1/np.sqrt(3), 1/np.sqrt(3), 1/np.sqrt(3)])):
        if 0 < ((l/dl)-int(l/dl)) < 1:
            p = np.zeros((3, int(l/dl)+2))
            o = np.zeros((1, int(l/dl)+2))
            a = np.zeros((1, int(l/dl)+2))
            v = np.zeros((3, int(l/dl)+2))
        else:
            p = np.zeros((3, int(l/dl)+1))
            o = np.zeros((1, int(l/dl)+1))
            a = np.zeros((1, int(l/dl)+1))
            v = np.zeros((3, int(l/dl)+1))
        p[:, 0] = FK.homogeneous(q)
        i = 0
        k = 0
        while i < l:
            k += 1
            p[:, k] = np.array([0.15+(((2*np.pi/l)*i)/20), 0.15+(0.25*np.cos((2*np.pi/l)*i)),
                                0.15+(0.25*np.sin((2*np.pi/l)*i))])
            o[:, k] = np.pi
            '''x axis = 120 ASCII'''
            a[:, k] = 120
            v[:, k] = z
            i = i+dl
        return p, o, a, v
