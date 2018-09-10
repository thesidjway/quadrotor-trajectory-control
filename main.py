from crazyflie import crazyflie
import controller
from qcutils import *

params = crazyflie()
print params

q = [0.7,0.5,0.5,1];
q = np.float32(q)

R = QuatToRot(q)
q = RotToQuat(R)

phi, theta, psi = RotToRPY_ZXY(R);
R = RPYtoRot_ZXY(phi,theta,psi);
