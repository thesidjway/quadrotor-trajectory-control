import numpy as np
from controller import controller
from trajectory import trajectory
from qcutils import *
from crazyflie import crazyflie


def quadEOM (t, s):
    params = crazyflie()
    s = np.transpose(np.float32([s]))
    qd = stateToQd(np.transpose(s));
    desired_state = trajectory(t);
    qd.pos_des      = desired_state.pos;
    qd.vel_des      = desired_state.vel;
    qd.acc_des      = desired_state.acc;
    qd.yaw_des      = desired_state.yaw;
    qd.yawdot_des   = desired_state.yawdot;
    [F, M, trpy, drpy] = controller(qd, t, params);
    sdot = get_sdot(t, s, F, M, params);
    return sdot

def get_sdot(t, s, F, M, params):
    A = np.float32([[0.25,  0, -0.5/params.arm_length],
         [0.25,  0.5/params.arm_length,  0],
         [0.25,  0,  0.5/params.arm_length],
         [0.25, -0.5/params.arm_length,  0]]);
    prop_thrusts = np.matmul(A, np.concatenate((F,M[0:2]),axis=0));
    np.putmask(prop_thrusts, prop_thrusts > params.maxF/4, params.maxF/4)
    np.putmask(prop_thrusts, prop_thrusts < params.minF/4, params.minF/4)
    prop_thrusts_clamped = prop_thrusts;

    B = np.float32([[1, 1, 1, 1],
        [0, params.arm_length, 0, -params.arm_length],
        [-params.arm_length, 0, params.arm_length, 0]]);

    F = np.matmul(B[0:1], prop_thrusts_clamped);
    temp1 = np.float32([[M[2,0]]]);
    M = np.concatenate((np.matmul(B[1:3], prop_thrusts_clamped), temp1), axis=0)

    x = s[0,0];
    y = s[1,0];
    z = s[2,0];
    xdot = s[3,0];
    ydot = s[4,0];
    zdot = s[5,0];
    qW = s[6,0];
    qX = s[7,0];
    qY = s[8,0];
    qZ = s[9,0];
    p = s[10,0];
    q = s[11,0];
    r = s[12,0];

    quat = np.float32([[qW], [qX], [qY], [qZ]]);
    bRw = QuatToRot(quat);
    wRb = np.linalg.inv(bRw);

    accel = 1 / params.mass * (np.matmul(wRb , np.float32([[0], [0], [F]])) - np.float32([[0], [0], [params.mass * params.grav]]));

    K_quat = 2;
    quaterror = 1 - (qW**2 + qX**2 + qY**2 + qZ**2);
    qdot = -1/2*np.matmul(
    np.float32([[0, -p, -q, -r],
             [p,  0, -r,  q],
             [q,  r,  0, -p],
             [r, -q,  p,  0]]) , quat) + K_quat*quaterror * quat;

    omega = np.float32([[p],[q],[r]]);
    omegaT = np.transpose(omega)
    temp1 = np.transpose(np.matmul(params.I , omega))
    pqrdot   = np.matmul(params.invI , (M - np.transpose(np.cross(omegaT, temp1))));
    sdot = np.zeros((13,1),dtype=float);
    sdot[0,0]  = xdot;
    sdot[1,0]  = ydot;
    sdot[2,0]  = zdot;
    sdot[3,0]  = accel[0];
    sdot[4,0]  = accel[1];
    sdot[5,0]  = accel[2];
    sdot[6,0]  = qdot[0,0];
    sdot[7,0]  = qdot[1,0];
    sdot[8,0]  = qdot[2,0];
    sdot[9,0] = qdot[3,0];
    sdot[10,0] = pqrdot[0];
    sdot[11,0] = pqrdot[1];
    sdot[12,0] = pqrdot[2];

    return sdot[:,0]
