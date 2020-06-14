from IK import *
import multiprocessing
from threading import *
from tkinter import *
from Paths import *
from Plot import *
from Velocity import *

D = Derivatives()
FK = ForwardKinematics()
IK = InverseKinematics()
P = Parameters()
Plot = Plot()
T = Paths()
V = Velocity()

q = np.zeros(P.links().shape[0])
signal = np.zeros(P.links().shape[0])
p = np.zeros(3)
o = np.zeros(1)
a = np.zeros(1)
v = np.zeros(3)
w = np.zeros(P.links().shape[0])
time = 0
dt = 0
counter = 0


class trunk:
    @staticmethod
    def algorithm(u, z):
        s = IK.compute(q, p, o, a, u, z)
        return s


class GUI:
    def __init__(self):
        self.root = Tk()
        self.root.title("Z Dynamics")
        self.root.columnconfigure(1, weight = 1)
        self.root.rowconfigure(1, weight = 1)

    def header(self):
        Label(self.root, text = 'zRobot 2.0', font = "Helvetica 30").grid(row = 0, columnspan = 5, sticky = 'NEW')
        GUI.selection()

    def selection(self):
        s = ['Choose kinematics type...', 'Forward Kinematics', 'Inverse Kinematics']
        z = StringVar()
        z.set(s[0])
        m = OptionMenu(self.root, z, *s)
        m.grid(row = 1, columnspan = 5, sticky = 'NEW')
        Button(self.root, text = 'GO',
               command = lambda: messagebox.showwarning("WARNING", "Please choose a valid option") if z.get() == s[
                   0] else GUI.forward() if z.get() == s[1] else GUI.inverse(), font = "Helvetica 15").grid(
            row = 2, columnspan = 5, sticky = 'NEW')

    @staticmethod
    def forward():
        GUI.reset()
        GUI.clear()
        GUI.sliders()

    def sliders(self, n = P.links().shape[0], r = 3, c = 0):
        z = list()
        for i in range(n):
            Label(self.root, text = 'Joint ' + str(i + 1) + ' [DEG]', font = "Helvetica 15").grid(row = r + i,
                                                                                                  column = c,
                                                                                                  sticky = 'WE')
            s = Scale(self.root, command = lambda x = 0: GUI.homogeneous(z, r, c), font = "Helvetica 15",
                      orient = HORIZONTAL, from_ = -360, to = 360, resolution = 0.01)
            s.grid(row = r + i, column = c + 1, sticky = 'NEW')
            s.set(0.01)
            s.set(0)
            z.append(s)

    def homogeneous(self, l, f, c):
        global q
        for i in range(len(l)):
            q[i] = l[i].get()
        q = q*(np.pi/180)
        r = FK.homogeneous(q)
        Label(self.root, text = 'Robot end-effector position [meters]', font = "Helvetica 15").grid(row = f,
                                                                                                    column = c + 2,
                                                                                                    sticky = 'NEW')
        z = ['x: ', 'y: ', 'z: ']
        for i in range(len(z)):
            Label(self.root, text = z[i] + str(r[i]), font = "Helvetica 15").grid(row = f + 1 + i, column = c + 2,
                                                                                  sticky = 'NEW')
        Button(self.root, text = "Send to robot", command = lambda: print(q), font = "Helvetica 15").grid(
            row = f + 1 + len(z), column = c + 2, sticky = 'NEW')

    @staticmethod
    def inverse():
        GUI.reset()
        GUI.clear()
        GUI.ikselection()

    def ikselection(self):
        s = ['Choose IK kinematics type...', 'Time Path', 'Spacial Path', 'Point to Point', 'Browse CSV File']
        z = StringVar()
        z.set(s[0])
        m = OptionMenu(self.root, z, *s)
        m.grid(row = 3, columnspan = 4, sticky = 'NEW')
        Button(self.root, text = 'GO',
               command = lambda: messagebox.showwarning("WARNING", "Please choose a valid option") if z.get() == s[
                   0] else GUI.timepath() if z.get() == s[1] else GUI.spatialpath() if z.get() == s[
                   2] else GUI.point2point() if z.get() == s[
                   3] else GUI.browsefile(), font = "Helvetica 15").grid(
            row = 4, columnspan = 4, sticky = 'NEW')

    def timepath(self):
        GUI.reset()
        GUI.clear()
        GUI.sliders(P.links().shape[0], 5, 0)
        GUI.timeentries('timepath', ['Total time to follow the path [sec]', 'Time-step size per point [sec]'], 5, 3)

    def spatialpath(self):
        GUI.reset()
        GUI.clear()
        GUI.sliders(P.links().shape[0], 5, 0)
        GUI.spatialmenu(5, 3)

    def point2point(self):
        GUI.reset()
        GUI.clear()
        GUI.sliders(P.links().shape[0], 5, 0)
        GUI.p2pmenu()

    def browsefile(self):
        GUI.reset()
        GUI.clear()
        GUI.sliders(P.links().shape[0], 5, 0)
        Button(self.root, text = "Browse CSV file",
               command = lambda: GUI.file(), font = "Helvetica 15").grid(row = 5, column = 3, sticky = 'NEW')

    def file(self):
        global p, o, a, v, dt
        p, o, a, v, dt = P.readfile()
        p[:, 0] = FK.homogeneous(q)
        dt = D.spatial(p, v)
        Thread(target = Plot.positionvelocity,
               args = (p, v, 'Point-to-point', 'p', 'Points')).start()
        GUI.solverandmatrix(6, 2)

    def p2pmenu(self):
        z = list()
        Label(self.root, text = 'Set number of points to reach', font = "Helvetica 15").grid(
            row = 5, column = 3,
            sticky = 'N')
        r = Entry(self.root, font = "Helvetica 15")
        r.grid(row = 6, column = 3, sticky = 'EW')
        r.insert(0, "0")
        z.append(r)
        Button(self.root, text = "Set number of points",
               command = lambda: GUI.p2pentries(z) if isinstance(
                   float(z[0].get()), float) and float(
                   z[0].get()) != 0 and float(z[0].get()) > 0 else messagebox.showwarning('WARNING',
                                                                                          'Please, introduce a valid positive integer number'),
               font = "Helvetica 15").grid(
            row = 7, column = 3, sticky = 'NEW')

    def p2pentries(self, z):
        def count(z, axis):
            global counter, p, o, a, v, dt
            if counter > float(z[0].get())-1:
                counter = 0
                messagebox.showwarning('WARNING',
                                       'All points have been set up. If you press <<Set point>> again, you will'
                                       ' reset your previous data')
                GUI.computepaths(0, 0, 'point2point', 0)
            elif counter < float(z[0].get()):
                x = z[1].get().split(",")
                y = z[3].get().split(",")
                p[:, counter+1] = np.array([float(x[0]), float(x[1]), float(x[2])])
                o[:, counter+1] = float(z[2].get())*(np.pi/180)
                a[:, counter+1] = ord(axis)
                v[:, counter+1] = np.array([float(y[0]), float(y[1]), float(y[2])])
                dt[counter+1] = float(z[4].get())
                counter += 1

        global p, o, a, v, dt
        p = np.zeros((3, int(z[0].get())+1))
        p[:, 0] = FK.homogeneous(q)
        o = np.zeros((1, int(z[0].get())+1))
        a = np.zeros((1, int(z[0].get())+1))
        v = np.zeros((3, int(z[0].get())+1))
        dt = np.zeros(int(z[0].get())+1)

        Label(self.root, text = '(x, y, z) coordinates [meters]', font = "Helvetica 15").grid(
            row = 8, column = 3,
            sticky = 'N')
        r = Entry(self.root, font = "Helvetica 15")
        r.grid(row = 9, column = 3, sticky = 'EW')
        r.insert(0, "0.0, 0.0, 0.0")
        z.append(r)
        s = ['Chose rotation axis and introduce rotation value...', 'X', 'Y', 'Z']
        u = StringVar()
        u.set(s[0])
        m = OptionMenu(self.root, u, *s)
        m.grid(row = 10, column = 3, sticky = 'NEW')
        r = Entry(self.root, font = "Helvetica 15")
        r.grid(row = 11, column = 3, sticky = 'EW')
        r.insert(0, "0.0")
        z.append(r)
        Label(self.root, text = '(Vx, Vy, Vz) End-effector velocity [m/s]\n(if time to reach the point is unknown)', font = "Helvetica 15").grid(
            row = 12, column = 3,
            sticky = 'N')
        r = Entry(self.root, font = "Helvetica 15")
        r.grid(row = 13, column = 3, sticky = 'EW')
        r.insert(0, "0.0, 0.0, 0.0")
        z.append(r)
        Label(self.root, text = 'Time to reach the point [sec]\n(if velocity is unknown)', font = "Helvetica 15").grid(
            row = 14, column = 3,
            sticky = 'N')
        r = Entry(self.root, font = "Helvetica 15")
        r.grid(row = 15, column = 3, sticky = 'EW')
        r.insert(0, "0.0")
        z.append(r)
        Button(self.root, text = "Set point data",
               command = lambda: count(z, u.get()) if u.get() != s[0] else messagebox.showwarning('WARNING',
                                                                                                'Please, choose a valid axis option'),
               font = "Helvetica 15").grid(row = 16, column = 3,
                                           sticky = 'NEW')

    def spatialmenu(self, f, c):
        z = list()
        Label(self.root, text = 'Total length to form the path [meters]', font = "Helvetica 15").grid(
            row = f, column = c,
            sticky = 'N')
        r = Entry(self.root, font = "Helvetica 15")
        r.grid(row = f + 1, column = c, sticky = 'EW')
        r.insert(0, "0.0")
        z.append(r)
        Label(self.root, text = 'Length-step size [meters]', font = "Helvetica 15").grid(
            row = f+2, column = c,
            sticky = 'N')
        g = Entry(self.root, font = "Helvetica 15")
        g.grid(row = f + 3, column = c, sticky = 'EW')
        g.insert(0, "0.0")
        z.append(g)
        s = ['Choose end-effector velocity option...', 'Total End-effector Velocity', 'Velocity per Axis']
        u = StringVar()
        u.set(s[0])
        m = OptionMenu(self.root, u, *s)
        m.grid(row = f+4, column = c, sticky = 'NEW')
        Button(self.root, text = "GO",
               command = lambda: messagebox.showwarning('WARNING',
                                                        'Please, choose a valid velocity option') if u.get() == s[
                   0] else GUI.total(f + 6, c, z) if u.get() == s[
                   1] and isinstance(float(z[0].get()), float) and isinstance(
                   float(z[1].get()), float) and float(z[0].get()) != 0 and float(z[1].get()) != 0 and float(
                   z[0].get()) > float(z[1].get()) else GUI.peraxis(z) if u.get() == s[
                   2] and isinstance(float(z[0].get()),
                                     float) and isinstance(
                   float(z[1].get()), float) and float(z[0].get()) != 0 and float(z[1].get()) != 0 and float(
                   z[0].get()) > float(z[1].get()) else
               messagebox.showwarning('WARNING',
                                      'Please, introduce valid options for total length and length-step size'),
               font = "Helvetica 15").grid(
            row = f + 5, column = c, sticky = 'NEW')

    def peraxis(self, z):
        z.append('A')
        Label(self.root, text = 'X axis', font = "Helvetica 15").grid(
            row = 11, column = 3,
            sticky = 'N')
        x = Entry(self.root, font = "Helvetica 15")
        x.grid(row = 12, column = 3, sticky = 'EW')
        x.insert(0, "0.0")
        z.append(x)

        Label(self.root, text = 'Y axis', font = "Helvetica 15").grid(
            row = 13, column = 3,
            sticky = 'N')
        y = Entry(self.root, font = "Helvetica 15")
        y.grid(row = 14, column = 3, sticky = 'EW')
        y.insert(0, "0.0")
        z.append(y)

        Label(self.root, text = 'Z axis', font = "Helvetica 15").grid(
            row = 15, column = 3,
            sticky = 'N')
        g = Entry(self.root, font = "Helvetica 15")
        g.grid(row = 16, column = 3, sticky = 'EW')
        g.insert(0, "0.0")
        z.append(g)
        Button(self.root, text = "GO",
               command = lambda: GUI.computepaths(0, 0, 'spatialpath', z) if isinstance(float(z[3].get()),
                                                                                        float) and isinstance(
                   float(z[4].get()),
                   float) and isinstance(
                   float(z[5].get()), float) and np.sqrt(
                   float(z[3].get()) ** 2 + float(z[4].get()) ** 2 + float(
                       z[5].get()) ** 2) <= 1 else messagebox.showwarning('WARNING',
                                                                          'End-effector velocity must be between -1 m/s and 1 m/s, excepting 0 m/s'),
               font = "Helvetica 15").grid(
            row = 17, column = 3, sticky = 'NEW')

    def total(self, f, c, z):
        Label(self.root, text = 'Total end-efector velocity [m/s]', font = "Helvetica 15").grid(
            row = f, column = c,
            sticky = 'N')
        z.append('T')
        r = Entry(self.root, font = "Helvetica 15")
        r.grid(row = f + 1, column = c, sticky = 'EW')
        r.insert(0, "0.0")
        z.append(r)
        s = ['Movement axes...', 'X', 'Y', 'Z', 'XY', 'XZ', 'YZ', 'XYZ']
        u = StringVar()
        u.set(s[0])
        m = OptionMenu(self.root, u, *s)
        m.grid(row = f+2, column = c, sticky = 'NEW')
        z.append(u)
        Button(self.root, text = "Compute desired trajectory and velocity",
               command = lambda: messagebox.showwarning('WARNING',
                                                        'End-effector velocity must be between -1 m/s and 1 m/s, excepting 0 m/s')
               if float(z[3].get()) < -1 or float(z[3].get()) == 0 or float(z[3].get()) > 1 else
               messagebox.showwarning('WARNING',
                                      'Please, choose a movement axes valid option')
               if u.get() == s[0] else GUI.computepaths(f, c, 'spatialpath', z),
               font = "Helvetica 15").grid(
            row = f + 3, column = c, sticky = 'NEW')

    def timeentries(self, t, s, f, c):
        z = list()
        for i in range(len(s)):
            Label(self.root, text = s[i], font = "Helvetica 15").grid(row = f if i == 0 else f+i+1, column = c, sticky = 'N')
            r = Entry(self.root, font = "Helvetica 15")
            r.grid(row = f+1 if i == 0 else f+3, column = c, sticky = 'EW')
            r.insert(0, "0.0")
            z.append(r)
        Button(self.root, text = "Compute desired trajectory and velocity", command = lambda: GUI.computepaths(f, c, t, z),
               font = "Helvetica 15").grid(
            row = f+4, column = c, sticky = 'NEW')

    def computepaths(self, f, c, t, l):
        global p, o, a, v, time, dt
        if 'timepath' in t:
            z = np.zeros(len(l))
            for i in range(len(l)):
                try:
                    z[i] = float(l[i].get())
                except ValueError:
                    z[i] = 0
            if z[0] == 0 and z[1] == 0:
                messagebox.showwarning("WARNING", "Please enter values for total time and time-step size.")
            elif z[0] != 0 and z[1] == 0:
                messagebox.showwarning("WARNING", "Please enter value for time-step size.")
            elif z[0] == 0 and z[1] != 0:
                messagebox.showwarning("WARNING", "Please enter value for total time.")
            else:
                time = z[0]
                dt = z[1]
                p, o, a, v = T.time(q, time, dt)
                dt = D.spatial(p, v)
                Thread(target = Plot.positionvelocity,
                       args = (p, v, 'Time-varying', 't', 'Time (units depend on your time ratio)')).start()
                GUI.solverandmatrix(f + 5, c - 1)
        elif 'spatialpath' in t:
            if l[2] == 'T':
                velocity = float(l[3].get())
                axes = l[4].get()
                if axes == 'X':
                    v = np.array([velocity, 0, 0])
                if axes == 'Y':
                    v = np.array([0, velocity, 0])
                if axes == 'Z':
                    v = np.array([0, 0, velocity])
                if axes == 'XY':
                    v = np.array([velocity*(1/np.sqrt(2)), velocity*(1/np.sqrt(2)), 0])
                if axes == 'XZ':
                    v = np.array([velocity*(1/np.sqrt(2)), 0, velocity*(1/np.sqrt(2))])
                if axes == 'YZ':
                    v = np.array([0, velocity*(1/np.sqrt(2)), velocity*(1/np.sqrt(2))])
                if axes == 'XYZ':
                    v = np.array([velocity*(1/np.sqrt(3)), velocity*(1/np.sqrt(3)), velocity*(1/np.sqrt(3))])
                p, o, a, v = T.spatial(q, float(l[0].get()), float(l[1].get()), v)
                dt = D.spatial(p, v)
                Thread(target = Plot.positionvelocity,
                       args = (p, v, 'Length-varying', 'dl', 'Length (units depend on your length ratio)')).start()
                GUI.solverandmatrix(f+5, c - 1)
            elif l[2] == 'A':
                for i in range(3, 5):
                    k = 0
                    if float(l[i].get()) != 0:
                        k += 1
                if k == 0:
                    messagebox.showwarning("WARNING", "Please introduce at least one velocity value.")
                else:
                    v = np.array([float(l[3].get()), float(l[4].get()), float(l[5].get())])
                    p, o, a, v = T.spatial(q, float(l[0].get()), float(l[1].get()), v)
                    dt = D.spatial(p, v)
                    Thread(target = Plot.positionvelocity,
                           args = (
                               p, v, 'Length-varying', 'dl', 'Length (units depend on your length ratio)')).start()
                    GUI.solverandmatrix(18, 2)
        elif 'point2point' in t:
            if np.count_nonzero(v) != 0 and np.count_nonzero(dt) != 0:
                messagebox.showwarning("WARNING",
                                       "You must introduce data for velocity or time, not for both at the same time")
            if np.count_nonzero(v) == 0 and np.count_nonzero(dt) == 0:
                messagebox.showwarning("WARNING",
                                       "You must introduce data for velocity or time, not for both at the same time")
            elif np.count_nonzero(v) != 0 and np.count_nonzero(dt) == 0:
                dt = D.spatial(p, v)
                Thread(target = Plot.positionvelocity,
                       args = (p, v, 'Point-to-point', 'p', 'Points')).start()
                GUI.solverandmatrix(18, 2)
            elif np.count_nonzero(v) == 0 and np.count_nonzero(dt) != 0:
                for j in range(1, dt.shape[0]):
                    v[:, j] = (p[:, j] - p[:, j - 1])/dt[j]
                    if np.sqrt((v[0, j]) ** 2 + (v[1, j]) ** 2 + (v[2, j]) ** 2) > 1:
                        k = np.count_nonzero(v[:, j])
                        for i in range(3):
                            if v[i, j] > 0:
                                v[i, j] = 1
                            elif v[i, j] < 0:
                                v[i, j] = -1
                        v[:, j] = v[:, j]*(1/np.sqrt(k))
                Thread(name = 'Position and Velocity', target = Plot.positionvelocity,
                       args = (p, v, 'Point-to-point', 'p', 'Points')).start()
                GUI.solverandmatrix(18, 2)

    def solverandmatrix(self, f, c):
        s = ['Choose solver type...', 'Continuous Solver', 'Discrete Solver']
        u = StringVar()
        u.set(s[0])
        m = OptionMenu(self.root, u, *s)
        m.grid(row = 10, column = c, sticky = 'NEW')
        j = ['Choose Jacobian matrix form...', 'Pseudoinverse Jacobian matrix', 'Transposed Jacobian matrix']
        z = StringVar()
        z.set(j[0])
        n = OptionMenu(self.root, z, *j)
        n.grid(row = 11, column = c, sticky = 'NEW')
        Button(self.root, text = "Compute Inverse Kinematics",
               command = lambda: Thread(target = GUI.start, args = (s, j, u.get(), z.get())).start(),
               font = "Helvetica 15").grid(
            row = f, column = c + 1, sticky = 'NEW')

    def start(self, s, j, u, z):
        elephant = trunk()
        global q, dt, signal, w
        if u != s[0]:
            u = u[0]
        else:
            u = 'd'
        if z != j[0]:
            z = z[0]
        else:
            z = 'p'
        if u == 'C' or z == 'T':
            messagebox.showwarning("Warning",
                                   'You must consider that, if this algorithm uses continuous solver or '
                                   'transposed Jacobian matrix, it will be slower than other options')
        bar = ttk.Progressbar(self.root, mode="indeterminate")
        bar.grid(row = int(P.links().shape[0]*0.5)+4, column = 4, sticky = 'EW')
        bar.start()
        g = multiprocessing.Pool(multiprocessing.cpu_count())
        s = g.apply(elephant.algorithm, args = (u, z))
        g.close()
        bar.stop()
        bar.destroy()
        if np.count_nonzero(s) != 0:
            w = V.algorithm(s, p, v, 'a')
            signal = IK.signal(s, dt, 0.003)
            signal = np.delete(signal, np.where(~signal.any(axis=0))[0], axis = 1)
            Thread(name = 'Joints and Velocity', target = Plot.signal,
                       args = (s, w, 'Time (units depend on your time ratio)')).start()
            Button(self.root, text = "START", command = lambda: print(signal),
                   font = "Helvetica 15").grid(row = 4, column = 4, rowspan = P.links().shape[0], sticky = 'NS')

    def clear(self):
        z = self.root.grid_slaves()
        for l in z:
            l.destroy()
        GUI.header()
        GUI.selection()

    @staticmethod
    def reset():
        global q, signal, p, o, a, v, w, time, dt
        q = np.zeros(P.links().shape[0])
        signal = np.zeros(P.links().shape[0])
        p = np.zeros(3)
        o = np.zeros(1)
        a = np.zeros(1)
        v = np.zeros(3)
        w = np.zeros(P.links().shape[0])
        time = 0
        dt = 0


GUI = GUI()
GUI.header()
mainloop()
