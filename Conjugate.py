import numpy as np


class Conjugate:

    @staticmethod
    def conjugate(v=np.zeros(8)):
        v[0] = v[0]
        v[1] = -v[1]
        v[2] = -v[2]
        v[3] = -v[3]
        v[4] = v[4]
        v[5] = -v[5]
        v[6] = -v[6]
        v[7] = -v[7]
        return v
