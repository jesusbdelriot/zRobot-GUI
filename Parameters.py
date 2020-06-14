import numpy as np
import os
from tkinter import filedialog
from tkinter import messagebox


class Parameters:

    @staticmethod
    def links():
        z = np.array([0.2075, 0.13455, 0.2615, 0.15039, 0.2615, 0.15039, 0.16558])
        return z

    @staticmethod
    def classical(q):
        p = Parameters()
        z = p.links()
        h = np.array([[q[0], z[0], 0, -np.pi/2],
                      [q[1], z[1], 0, np.pi/2],
                      [q[2], z[2], 0, np.pi/2],
                      [q[3], z[3], 0, -np.pi/2],
                      [q[4], z[4], 0, -np.pi/2],
                      [q[5], z[5], 0, +np.pi/2],
                      [q[6], z[6], 0, 0]])
        return h

    @staticmethod
    def modified(q):
        p = Parameters()
        z = p.links()
        h = np.array([[00000000, 0, q[0], z[0]],
                      [-np.pi/2, 0, q[1], z[1]],
                      [+np.pi/2, 0, q[2], z[2]],
                      [+np.pi/2, 0, q[3], z[3]],
                      [-np.pi/2, 0, q[4], z[4]],
                      [-np.pi/2, 0, q[5], z[5]],
                      [+np.pi/2, 0, q[6], z[6]]])
        return h

    @staticmethod
    def readfile():
        z = list()
        d = filedialog.askopenfilename(initialdir = os.getcwd(), title = "Select CSV file",
                                       filetypes = (("Text files", "*.txt"), ("all files", "*.*")))
        with open(d, 'r') as f:
            for k in f:
                z.append(k.split(','))
            p = np.zeros((3, len(z)))
            o = np.zeros((1, len(z)))
            a = np.zeros((1, len(z)))
            v = np.zeros((3, len(z)))
            dt = np.zeros(len(z))
            if z[0][len(z[0])-1].strip() == 'time':
                for i in range(1, len(z)):
                    p[:, i] = np.array([float(z[i][0].strip()), float(z[i][1].strip()), float(z[i][2].strip())])
                    o[:, i] = float(z[i][3].strip())*(np.pi/180)
                    a[:, i] = ord(z[i][4].strip())
                    dt[i] = float(z[i][5].strip())
                for j in range(1, dt.shape[0]):
                    v[:, j] = (p[:, j]-p[:, j-1])/dt[j]
                    if np.sqrt((v[0, j])**2+(v[1, j])**2+(v[2, j])**2) > 1:
                        k = np.count_nonzero(v[:, j])
                        for i in range(3):
                            if v[i, j] > 0:
                                v[i, j] = 1
                            elif v[i, j] < 0:
                                v[i, j] = -1
                        v[:, j] = v[:, j]*(1/np.sqrt(k))
            elif z[0][len(z[0])-3].strip() == 'Vx' and z[0][len(z[0])-2].strip() == 'Vy' and z[0][len(z[0])-1].strip() == 'Vz':
                for i in range(1, len(z)):
                    p[:, i] = np.array([float(z[i][0].strip()), float(z[i][1].strip()), float(z[i][2].strip())])
                    o[:, i] = float(z[i][3].strip())*(np.pi/180)
                    a[:, i] = ord(z[i][4].strip())
                    v[:, i] = np.array([float(z[i][5].strip()), float(z[i][6].strip()), float(z[i][7].strip())])
            else:
                messagebox.showwarning("WARNING", "File error, please check if your file data is correct")
        return p, o, a, v, dt
