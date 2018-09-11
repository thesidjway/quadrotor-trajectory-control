from crazyflie import crazyflie
from controller import controller
from qcutils import *
from trajectory import trajectory
from quadrotor import quadEOM

params = crazyflie()
print params

q = [0.7,0.5,0.5,1];
q = np.float32(q)

R = QuatToRot(q)
q = RotToQuat(R)

phi, theta, psi = RotToRPY_ZXY(R);
R = RPYtoRot_ZXY(phi,theta,psi);

x = np.ones((1,13), dtype=float);
qd = stateToQd(x)
xdash = qdToState(qd)

s = init_state([2,3,4],2)
print "s: ", s

quadEOM(5,s,params)
