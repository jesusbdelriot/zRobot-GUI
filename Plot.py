import matplotlib.pyplot as plt


class Plot:

    @staticmethod
    def positionvelocity(p, v, t, k, x):
        plt.clf()
        plt.subplot(211)
        plt.title("End-effector estimated Position and Velocity for " + t + " Path")
        plt.plot(p[0, :], label = 'x(' + k + ')')
        plt.plot(p[1, :], label = 'y(' + k + ')')
        plt.plot(p[2, :], label = 'z(' + k + ')')
        plt.ylabel('P(' + k + ') [meters]')
        plt.grid(True)
        plt.legend()
        plt.subplot(212)
        plt.plot(v[0, :], label = r'v$_x$(' + k + ')')
        plt.plot(v[1, :], label = r'v$_y$(' + k + ')')
        plt.plot(v[2, :], label = r'v$_z$(' + k + ')')
        plt.xlabel(x)
        plt.ylabel('V(' + k + r') $\left[\frac{meters}{second}\right]$')
        plt.grid(True)
        plt.legend()
        plt.show()

    @staticmethod
    def signal(s, w, x):
        plt.clf()
        plt.subplot(211)
        plt.title("Joint positions and its angular velocities")
        for i in range(s.shape[0]):
            plt.plot(s[i, :], label = r'$\theta_' + str(i+1) + r'$(t)')
        plt.ylabel(r'$\theta$(t) [rad]')
        plt.grid(True)
        plt.legend()
        plt.subplot(212)
        for j in range(w.shape[0]):
            plt.plot(w[j, :], label = r'$\omega_' + str(j+1) + r'$(t)')
        plt.xlabel(x)
        plt.ylabel(r'$\omega$(t) $\left[\frac{rad}{second}\right]$')
        plt.grid(True)
        plt.legend()
        plt.show()
