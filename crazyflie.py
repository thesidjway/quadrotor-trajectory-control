import numpy as np

class Params:
    mass = 0;
    I = np.ones((3, 3), dtype=float)
    L = 0
    invI = np.ones((3, 3), dtype=float)
    arm_length = 0
    maxangle = 0
    maxF = 0
    minF = 0


def crazyflie():
    m = 0.030;
    g = 9.81;  #gravitational constant
    I = [[1.43e-5,   0,          0], #inertial tensor in m^2 kg
          [0,         1.43e-5,    0],
          [0,         0,          2.89e-5]];
    I = np.float32(I)
    L = 0.046; #arm length in m
    params = Params();
    params.mass = m;
    params.I    = I;
    params.invI = np.linalg.inv(I);
    params.grav = g;
    params.arm_length = L;
    params.maxangle = 40*np.pi/180;
    params.maxF     = 2.5*m*g;
    params.minF     = 0.05*m*g;
    return params;
