import numpy as np
import math

def controller(qd, t, params):
    Kp = [[15],[15],[30]];
    Kd =  [[12],[12],[10]];
    KpM = [[3000],[3000],[3000]];
    KdM = [[300],[300],[300]];
    Kp = np.float32(Kp);
    Kd = np.float32(Kd);
    KpM = np.float32(KpM);
    KdM = np.float32(KdM);

    print "qd.acc_des", qd.acc_des
    print "qd.vel_des", qd.vel_des
    print "qd.vel", qd.vel
    print "qd.pos_des", qd.pos_des
    print "qd.pos", qd.pos

    acc_des = qd.acc_des + Kd * (qd.vel_des - qd.vel) + Kp * (qd.pos_des - qd.pos);
    phi_des = 1/params.grav * (acc_des[0] * math.sin(qd.yaw_des) - acc_des[1] * math.cos(qd.yaw_des));
    theta_des = 1/params.grav * (acc_des[0] * math.cos(qd.yaw_des) + acc_des[1] * math.sin(qd.yaw_des));
    psi_des = qd.yaw_des;
    euler_des = [[phi_des], [theta_des], [psi_des]];
    euler_des = np.float32(euler_des)
    pqr_des = [[0], [0], [qd.yawdot_des]];
    #force
    print "dafuq", acc_des
    F  = params.mass * (params.grav + acc_des[2]);
    #moment
    M =  np.matmul(params.I , (KdM * (pqr_des - qd.omega) + KpM * (euler_des - qd.euler)));
    trpy = [F, phi_des, theta_des, psi_des];
    trpy = np.float32(trpy);
    drpy = [0,0,0,0];
    drpy = np.float32(drpy);

    print "MOHA TEST F: ", F
    print "M: ", M
    print "trpy: ", trpy
    print "drpy: ", drpy
    return F, M, trpy, drpy
