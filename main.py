import sys
from scipy.integrate import solve_ivp
sys.dont_write_bytecode = True
from crazyflie import crazyflie
from controller import controller
from qcutils import *
from trajectory import trajectory
from quadrotor import quadEOM
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

s0 = init_state(trajectory(0).pos,0)

sol = solve_ivp(quadEOM, [0, 20], s0[:,0])
np.set_printoptions(threshold=np.inf)
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(sol.y[0,:], sol.y[1,:], sol.y[2,:])

plt.show()
