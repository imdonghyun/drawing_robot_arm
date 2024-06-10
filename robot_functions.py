import os
import sys

import numpy as np
import matplotlib.pyplot as plt
import time

sys.path.append("../")

from src.utils import *
from src.core.pybullet_core import PybulletCore

from neuromeka import IndyDCP3
from time import sleep, time, perf_counter

from IPython.display import clear_output
import json

indy = IndyDCP3(robot_ip='192.168.0.13', index=0)

pb = PybulletCore()
pb.connect(robot_name = "indy7_v2", joint_limit=True, constraint_visualization = False)


def sync_indy():
    q = indy.get_control_data()['q']
    pb.MoveRobot(q, degree=True)
    return

def wait_indy():
    while True:
        sync_indy()
        if not indy.get_motion_data()["is_in_motion"]:
            break
        #time.sleep(0.1)
        sleep(0.001)
    PRINT_BLACK("Move done!!")
    return

def movej_indy(q, degree=True, blending_type=0):
    q = np.asarray(q).reshape(-1)
    if not degree:
        q = q * 180/np.pi
        
    indy.movej(q, blending_type=blending_type, acc_ratio=50)
    pb.MoveRobot(q, degree=True)
    return

# def movec_indy(T, blending_type=0):
    
#     p = SE3    
#     indy.movej(q, blending_type=blending_type, acc_ratio=50)
#     #pb.MoveRobot(q, degree=True)
#     return

def polynomial_time_scaling(_step, _total_steps):
    m1 = np.array([[1, 0, 0, 0],
                  [1, _total_steps, _total_steps**2, _total_steps**3],
                  [0, 1, 0, 0],
                  [0, 1, 2*_total_steps, 3*_total_steps**2]])
    m2 = np.array([[0],
                   [1],
                   [0],
                   [0]])
    a = np.linalg.inv(m1)@m2
    t = np.array([[1],
                  [_step+1],
                  [(_step+1)**2],
                  [(_step+1)**3]])
    return (a.T@t)[0,0]

def path_task_space_decoupled(_theta_initial, _T_final, _s, _q_i):
    
    # reshape theta
    theta_initial = np.reshape(_theta_initial*np.pi/180,(6,1))
    
    # forward kinematics
    _T_initial = pb.my_robot.pinModel.FK(theta_initial)
    
    # path caculation
    target_p = np.array([_T_initial[0:3,3] + _s*(_T_final[0:3,3] - _T_initial[0:3,3])]).T
    target_R = _T_initial[0:3,0:3]@Vec2Rot(Rot2Vec(_T_initial[0:3,0:3].T@_T_final[0:3,0:3])*_s)
    target_T = np.concatenate((target_R, target_p),axis=1)
    target_T = np.concatenate((target_T, [[0,0,0,1]]),axis=0)
    
    # jacobian calculation
    T_i = pb.my_robot.pinModel.FK(_q_i)
    Jb_i = pb.my_robot.pinModel.Jb(_q_i)
    R_i = T_i[0:3, 0:3]
    A_upper = np.concatenate((np.zeros([3, 3]), R_i), axis=1)
    A_lower = np.concatenate((np.eye(3), np.zeros([3, 3])), axis=1)
    A = np.concatenate((A_upper, A_lower), axis=0)
    Jv_i = A @ Jb_i
    
    # numerical inverse kinematics with matrix logarithm
    p_err = SE32PoseVec(target_T)-SE32PoseVec(T_i)
    target_joint_angle = _q_i + 0.01*np.linalg.inv(Jv_i)@p_err
    
    return target_T, target_joint_angle

def trajectory_making(_T_final, _t_f, _freq):
    
    # move to initial position
    sync_indy()
    sleep(2)
    q_i = pb.my_robot.q
    _theta_initial = q_i/np.pi*180

    # initialization for visualization
    debug_frames = [_T_final]
    pb.add_debug_frames([_T_final])
    trajectory = []

    # store initial timestamp
    t_initial = perf_counter()

    # moving
    for step in range(_freq*_t_f):

        # time scaling
        s = polynomial_time_scaling(step, _freq*_t_f)

        # path calculation
        T_i, q_i = path_task_space_decoupled(_theta_initial, _T_final, s, q_i)
        trajectory.append(T_i)
        pb.MoveRobot(q_i, degree=False)
        
        # visualize
        if(step%int(_freq*_t_f/50)==0):
            debug_frames.append(pb.my_robot.T_end)
            pb.add_debug_frames(debug_frames)

        # delay for constant frequency
        
        while True:
            t = perf_counter() - t_initial
            if(t >= (step+1)/_freq):
                break
            sleep(0)

    # visualize last position
    debug_frames.append(pb.my_robot.T_end)
    pb.add_debug_frames(debug_frames)

    # print total time taken while moving
    print("total time: ", t)
    
    return trajectory

def move_robot(_trajectories, _freq):

    indy.start_teleop(0)
    sleep(1)

    t_initial = perf_counter()
    
    for step in range(len(_trajectories)):
        T_des = _trajectories[step]
        p_des = np.zeros(6)
        p_des[0:3] = 1000*T_des[0:3, 3]
        p_des[3:6] = Rot2eul(T_des[0:3, 0:3], seq='XYZ', degree=True)
        
        indy.movetelel_abs(p_des, vel_ratio=0.5, acc_ratio=1)
        
        
        while True:
            t = perf_counter() - t_initial
            if(t >= (step+1)/_freq):
                break
            sleep(0)
    print("total time: ",t)
    wait_indy()
    indy.stop_teleop()
    
def move_to_original_base():
    q_i = np.array([0, -15, -75, 0, -90, 0])*np.pi/180
    movej_indy(q_i, degree=False)
    wait_indy()
    T_i = pb.my_robot.pinModel.FK(q_i)
    p_i = indy.get_control_data()["p"]
    T_i_from_indy = xyzeul2SE3([x/1000 for x in p_i[0:3]], [x for x in p_i[3:6]], degree=True, seq='XYZ')
    pb.add_debug_frames([T_i])
    
def move_to_photo_base():
    q_i = np.array([ 2.7561, 0.04 , -2.0154, -0.0028, -1.2183, 0.9536])
    movej_indy(q_i, degree=False)
    wait_indy()
    T_i = pb.my_robot.pinModel.FK(q_i)
    p_i = indy.get_control_data()["p"]
    T_i_from_indy = xyzeul2SE3([x/1000 for x in p_i[0:3]], [x for x in p_i[3:6]], degree=True, seq='XYZ')
    pb.add_debug_frames([T_i])
def move_to_draw_base():
    #q_i = np.array([ 1.5759, -0.7465, -1.8064, -0.    , -0.5888, 1.5758])
    q_i = np.array([ 1.5758, -0.6375, -1.7788, -0.    ,-0.7257, 1.5758])
    movej_indy(q_i, degree=False)
    wait_indy()
    T_i = pb.my_robot.pinModel.FK(q_i)
    p_i = indy.get_control_data()["p"]
    T_i_from_indy = xyzeul2SE3([x/1000 for x in p_i[0:3]], [x for x in p_i[3:6]], degree=True, seq='XYZ')
    pb.add_debug_frames([T_i])
def move_to_base():
    q_i = np.array([90, 0, -90, 0, -90, 90])*np.pi/180
    movej_indy(q_i, degree=False)
    wait_indy()
    T_i = pb.my_robot.pinModel.FK(q_i)
    p_i = indy.get_control_data()["p"]
    T_i_from_indy = xyzeul2SE3([x/1000 for x in p_i[0:3]], [x for x in p_i[3:6]], degree=True, seq='XYZ')
    pb.add_debug_frames([T_i])
    
def height_interpolation(_x_pixel, _y_pixel):
    with open('calibration_config.json') as json_file:
        pen_height_data = json.load(json_file)
    pen_height_1 = pen_height_data["pen_height_1"]
    pen_height_2 = pen_height_data["pen_height_2"]
    pen_height_3 = pen_height_data["pen_height_3"]
    pen_height_4 = pen_height_data["pen_height_4"]
    width_pixel = pen_height_data["width_pixel"]
    height_pixel = pen_height_data["height_pixel"]

    h_inter = pen_height_1 + (pen_height_2-pen_height_1)/width_pixel*(_x_pixel) + (pen_height_4-pen_height_1)/height_pixel*(_y_pixel)
    return h_inter



