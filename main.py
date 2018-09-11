import sys
from scipy.integrate import solve_ivp
sys.dont_write_bytecode = True
from crazyflie import crazyflie
from controller import controller
from qcutils import *
from trajectory import spiral, figure_of_8
from quadrotor import quadEOM
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

s0 = init_state(spiral(0).pos, 0)
sol = solve_ivp(quadEOM, [0,20] , s0[:,0])
np.set_printoptions(threshold=np.inf)
fig = plt.figure()
ax = fig.gca(projection='3d')
trajectory = []
for i in sol.t:
    trajectory.append(spiral(i).pos);
trajectory = np.float32(trajectory)[:,:,0]
#print trajectory
ax.plot(sol.y[0,:], sol.y[1,:], sol.y[2,:])
ax.plot(trajectory[:,0], trajectory[:,1], trajectory[:,2])
plt.gca().set_aspect('equal', adjustable='box')
plt.show()
