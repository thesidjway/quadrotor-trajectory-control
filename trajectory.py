import numpy as np
import math
from qcutils import *


def tj_from_line(start_pos, end_pos, time_ttl, t_c):
    pos = np.float32([[0],[0],[0]]);
    vel = np.float32([[0],[0],[0]]);
    acc = np.float32([[0],[0],[0]]);
    v_max = (end_pos-start_pos)*2/time_ttl;
    if t_c >= 0 and t_c < time_ttl/2:
        vel = v_max*t_c/(time_ttl/2);
        pos = start_pos + t_c*vel/2;
        acc = np.float32([[0],[0],[0]]);
    else:
        vel = v_max*(time_ttl-t_c)/(time_ttl/2);
        pos = end_pos - (time_ttl-t_c)*vel/2;
        acc = np.float32([[0],[0],[0]]);
    return pos, vel, acc

def pos_from_angle(a,radius):
    pos = np.float64([[radius*math.cos(a)],[radius*math.sin(a)],[2.5*a/(2*np.pi)]]);
    return pos

def get_vel(t, time_tol, radius, dt):
    angle1, _, _ = tj_from_line(0, 2*np.pi, time_tol, t);
    pos1 = pos_from_angle(angle1, radius);
    angle2, _, _ = tj_from_line(0, 2*np.pi, time_tol, t+dt);
    vel = (pos_from_angle(angle2, radius) - pos1)/dt;
    return vel


def spiral(t):
    desired_state = Qd()
    time_tol = 12;
    radius = 5;
    dt = 0.0001;

    if t > time_tol:
        pos = np.float32([[radius], [0], [2.5]]);
        vel = np.float32([[0],[0],[0]]);
        acc = np.float32([[0],[0],[0]]);
    else:
        angle, _, _ = tj_from_line(0, 2*np.pi, time_tol, t);
        pos = pos_from_angle(angle, radius);
        vel = get_vel(t, time_tol, radius, dt);
        acc = (get_vel(t+dt, time_tol, radius, dt) - get_vel(t, time_tol, radius, dt))/dt;


    yaw = 0;
    yawdot = 0;
    desired_state.pos = pos;
    desired_state.vel = vel;
    desired_state.acc = acc;
    desired_state.yaw = yaw;
    desired_state.yawdot = yawdot;
    return desired_state

def get_vel_fig8(t, time_tol, radius, dt):
    angle1, _, _ = tj_from_line_fig8(0, 4*np.pi, time_tol, t);
    angle1 = (angle1 - np.pi )/ 2;
    pos1 = pos_from_angle_fig8(angle1, radius);
    angle2, _, _ = tj_from_line_fig8(0, 4*np.pi, time_tol, t+dt);
    angle2 = (angle2 - np.pi )/ 2;
    pos2 = pos_from_angle_fig8(angle2, radius)
    vel = (pos2 - pos1)/dt;
    return vel

def pos_from_angle_fig8(a, radius):
    pos = np.float64([[-radius*math.cos(a)],[radius*math.sin(a)*math.cos(a)],[0]]);
    return pos

def tj_from_line_fig8(start_pos, end_pos, time_ttl, t_c):
    pos = np.float32([[0],[0],[0]]);
    vel = np.float32([[0],[0],[0]]);
    acc = np.float32([[0],[0],[0]]);
    v_max = (end_pos-start_pos)*2/time_ttl;
    vel = (end_pos-start_pos)/time_ttl;
    # if t_c >= 0 and t_c < time_ttl/2:
    #     vel = v_max*t_c/(time_ttl/2);
    #     pos = start_pos + t_c*vel/2;
    #     acc = np.float32([[0],[0],[0]]);
    # else:
    #     vel = v_max*(time_ttl-t_c)/(time_ttl/2);
    #     pos = end_pos - (time_ttl-t_c)*vel/2;
    #     acc = np.float32([[0],[0],[0]]);
    pos =  start_pos + t_c*vel;
    return pos, vel, acc

def figure_of_8(t):
    desired_state = Qd()
    time_tol = 20
    radius = 4
    dt = 0.0001;
    if t > time_tol:
        pos = np.float32([[0],[0],[0]]);
        vel = np.float32([[0],[0],[0]]);
        acc = np.float32([[0],[0],[0]]);
    else:
        angle, _, _ = tj_from_line_fig8(0, 4*np.pi, time_tol, t);
        angle = (angle - np.pi )/ 2;
        pos = pos_from_angle_fig8(angle, radius);
        vel = get_vel_fig8(t, time_tol, radius, dt);
        acc = (get_vel_fig8(t+dt, time_tol, radius, dt) - get_vel_fig8(t, time_tol, radius, dt))/dt;
    yaw = 0;
    yawdot = 0;
    desired_state.pos = pos;
    desired_state.vel = vel;
    desired_state.acc = acc;
    desired_state.yaw = yaw;
    desired_state.yawdot = yawdot;
    return desired_state
