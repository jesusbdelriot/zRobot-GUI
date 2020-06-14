import numpy as np


class Integral:

    @staticmethod
    def function(a, f, t):
        k1 = a
        k2 = a+(0.5*t*k1)
        k3 = a+(0.5*t*k2)
        k4 = a+(t*k3)
        f = f+((1/6)*(k1+(2*k2)+(2*k3)+k4)*t)
        return f

    @staticmethod
    def matrix(a, m, t):
        p = np.shape(a)[0]
        q = np.shape(a)[1]
        z = np.zeros((p, q))
        for i in range(p):
            for j in range(q):
                k1 = a[i, j]
                k2 = a[i, j]+(0.5*t*k1)
                k3 = a[i, j]+(0.5*t*k2)
                k4 = a[i, j]+(t*k3)
                z[i, j] = m[i, j]+((1/6)*(k1+(2*k2)+(2*k3)+k4)*t)
        return z

    @staticmethod
    def vector(a, v, t):
        p = np.shape(a)[0]
        z = np.zeros(p)
        for i in range(p):
            k1 = a[i]
            k2 = a[i]+(0.5*t*k1)
            k3 = a[i]+(0.5*t*k2)
            k4 = a[i]+(t*k3)
            z[i] = v[i]+((1/6)*(k1+(2*k2)+(2*k3)+k4)*t)
        return z
