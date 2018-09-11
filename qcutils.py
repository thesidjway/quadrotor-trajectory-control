import numpy as np
import math

class Qd:
    pos = np.zeros((3,1), dtype=float)
    vel = np.zeros((3,1), dtype=float)
    euler = np.zeros((3,1), dtype=float)
    omega = np.zeros((3,1), dtype=float)
    pos_des = np.zeros((3,1), dtype=float)
    vel_des = np.zeros((3,1), dtype=float)
    acc_des = np.zeros((3,1), dtype=float)
    yaw_des = np.zeros((3,1), dtype=float)
    yawdot_des = np.zeros((3,1), dtype=float)


def init_state( start, yaw ):
    s = np.zeros((13, 1), dtype=float)
    phi0   = 0.0;
    theta0 = 0.0;
    psi0   = yaw;
    Rot0   = RPYtoRot_ZXY(phi0, theta0, psi0);
    Quat0  = RotToQuat(Rot0);
    s[0,0] = start[0]; #x
    s[1,0] = start[1]; #y
    s[2,0] = start[2]; #z
    s[3,0] = 0; #xdot
    s[4,0] = 0; #ydot
    s[5,0] = 0; #zdot
    s[6,0]  = Quat0[0]; #qw
    s[7,0]  = Quat0[1]; #qx
    s[8,0]  = Quat0[2]; #qy
    s[9,0]  = Quat0[3]; #qz
    s[10,0] = 0; #p
    s[11,0] = 0; #q
    s[12,0] = 0; #r
    return s;

def qdToState(qd):
    x = np.zeros((1,13), dtype=float);
    x[0,0:3] = np.transpose(qd.pos);
    x[0,3:6] =  np.transpose(qd.vel);
    Rot = RPYtoRot_ZXY(qd.euler[0], qd.euler[1], qd.euler[2]);
    quat = RotToQuat(Rot);
    x[0,6:10] = quat;
    x[0,10:13] = np.transpose(qd.omega);
    return x;

def stateToQd(x):
    qd = Qd()

    qd.pos = np.float32([[x[0,0]], [x[0,1]], [x[0,2]]]);
    qd.vel = np.float32([[x[0,3]], [x[0,4]], [x[0,5]]]);
    Rot = QuatToRot(x[0,6:10]);
    [phi,theta,yaw] = RotToRPY_ZXY(Rot)
    qd.euler = np.float32([[phi], [theta], [yaw]]);
    qd.omega = np.float32([[x[0,10]], [x[0, 11]], [x[0, 12]]]);
    return qd


def QuatToRot(q):
    q = q/np.linalg.norm(q);
    qahat = np.zeros((3,3),dtype=float)
    qahat[0,1] = -q[3]
    qahat[0,2] = q[2]
    qahat[1,2] = -q[1]
    qahat[1,0] = q[3]
    qahat[2,0] = -q[2]
    qahat[2,1] = q[1]
    R = np.eye(3) + 2 * np.matmul(qahat, qahat) + 2 * q[0] * qahat;
    return R


def RotToQuat(R):
    tr = R[0,0] + R[1,1] + R[2,2];
    qw = 1;
    qx = 0;
    qy = 0;
    qz = 0;
    if (tr > 0):
        S = math.sqrt(tr+1.0) * 2;
        qw = 0.25 * S;
        qx = (R[2,1] - R[1,2]) / S;
        qy = (R[0,2] - R[2,0]) / S;
        qz = (R[1,0] - R[0,1]) / S;
    elif ((R[0,0] > R[1,1]) and (R[0,0] > R[2,2])):
        S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2;
        qw = (R[2,1] - R[1,2]) / S;
        qx = 0.25 * S;
        qy = (R[0,1] + R[1,0]) / S;
        qz = (R[0,2] + R[2,0]) / S;
    elif (R[1,1] > R[2,2]):
        S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2;
        qw = (R[0,2] - R[2,0]) / S;
        qx = (R[0,1] + R[1,0]) / S;
        qy = 0.25 * S;
        qz = (R[1,2] + R[2,1]) / S;
    else:
        S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2;
        qw = (R[1,0] - R[0,1]) / S;
        qx = (R[0,2] + R[2,0]) / S;
        qy = (R[1,2] + R[2,1]) / S;
        qz = 0.25 * S;
    q = [qw,qx,qy,qz];
    q = np.float32(q);
    q = q*np.sign(qw);
    return q;


def RotToRPY_ZXY(R):
    phi = math.asin(R[1,2]);
    psi = math.atan2(-R[1,0]/math.cos(phi),R[1,1]/math.cos(phi));
    theta = math.atan2(-R[0,2]/math.cos(phi),R[2,2]/math.cos(phi));
    return phi, theta, psi

def RPYtoRot_ZXY(phi,theta,psi):
    R = [[math.cos(psi)*math.cos(theta) - math.sin(phi)*math.sin(psi)*math.sin(theta),
    math.cos(theta)*math.sin(psi) + math.cos(psi)*math.sin(phi)*math.sin(theta),
    -math.cos(phi)*math.sin(theta)],
    [-math.cos(phi)*math.sin(psi),
    math.cos(phi)*math.cos(psi),
    math.sin(phi)],
    [math.cos(psi)*math.sin(theta) + math.cos(theta)*math.sin(phi)*math.sin(psi),
    math.sin(psi)*math.sin(theta) - math.cos(psi)*math.cos(theta)*math.sin(phi),
    math.cos(phi)*math.cos(theta)]];
    R = np.float32(R)
    return R
