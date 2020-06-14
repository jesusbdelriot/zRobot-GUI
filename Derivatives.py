import time
from Parameters import *

P = Parameters()


class Derivatives:

    @staticmethod
    def time(f, dt=time.clock()):
        m, n = f.shape
        d = np.zeros((m, n))
        for j in range(1, n-1):
            d[:, j] = (f[:, j+1]-f[:, j-1])/(2*dt)
            if np.sqrt((d[0, j]) ** 2 + (d[1, j]) ** 2 + (d[2, j]) ** 2) > 1:
                k = np.count_nonzero(d[:, j])
                for i in range(3):
                    if d[i, j] > 0:
                        d[i, j] = 1
                    elif d[i, j] < 0:
                        d[i, j] = -1
                d[:, j] = d[:, j]*(1/np.sqrt(k))
        return d

    @staticmethod
    def spatial(p=np.zeros((3, 2)), v=np.zeros((3, 2))):
        n = p.shape[1]
        dt = np.zeros(n)
        for j in range(1, n):
            dt[j] = np.sqrt(
                            ((p[0, j]-p[0, j-1])**2+(p[1, j]-p[1, j-1])**2+(p[2, j]-p[2, j-1])**2))/np.sqrt(
                            (v[0, j]**2+v[1, j]**2+v[2, j]**2))
            if np.isnan(dt[j]) or np.isinf(dt[j]):
                dt[j] = 0
            elif dt[j] == 0:
                dt[j] = dt[j-1]
        return dt
