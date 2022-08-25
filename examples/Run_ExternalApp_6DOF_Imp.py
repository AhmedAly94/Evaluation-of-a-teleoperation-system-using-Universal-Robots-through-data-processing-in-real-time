import sys

sys.path.append('')
import logging
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

import time
from matplotlib import pyplot as plt
from Functions import get_Traj, pose_scal, get_pose, dampingWrench, getSkewSymmetric
import numpy as np

from WrenchCalculation_Algorithm3 import WrenchForImpedance


# from Admittance_Mode import WrenchForImpedance


def list_to_setp_initcfg(setp, list):
    for i in range(0, 6):
        setp.__dict__["input_double_register_%i" % i] = list[i]
    return setp


def list_to_setp_wrench(setp, list):
    setp.input_double_register_6 = list[0]
    setp.input_double_register_7 = list[1]
    setp.input_double_register_8 = list[2]
    setp.input_double_register_9 = list[3]
    setp.input_double_register_10 = list[4]
    setp.input_double_register_11 = list[5]
    return setp


# ------------- robot communication stuff -----------------
ROBOT_HOST = '134.28.124.86'  # specify the IP-address of the robot
ROBOT_PORT = 30004  # specify the controller's port for the communication, 30004 stands for RTDE
config_filename = 'impedane_control_configuration.xml'  # specify xml file for data synchronization

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe('state')  # Define recipe for access to robot output ex. joints,tcp etc.
setp_names, setp_types = conf.get_recipe('setp')  # Define recipe for access to robot input
watchdog_names, watchdog_types = conf.get_recipe('watchdog')

# -------------------- Establish connection--------------------
con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)  # use the package rtde and pass the data into it
connection_state = con.connect()  # establish the connection

print("---------------Successfully connected to the robot-------------\n")

# get controller version
con.get_controller_version()

# ------------------- setup recipes ----------------------------
FREQUENCY = 500  # send data in 500 Hz instead of default 125Hz
con.send_output_setup(state_names, state_types, FREQUENCY)
setp = con.send_input_setup(setp_names, setp_types)  # Configure an input package that the
# external application will send to the robot controller
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

# input_double_register_24 - 29 represent the generated Path or the Target Position
setp.input_double_register_0 = 0
setp.input_double_register_1 = 0
setp.input_double_register_2 = 0
setp.input_double_register_3 = 0
setp.input_double_register_4 = 0
setp.input_double_register_5 = 0
# input_double_register_30 - 35 represent the calculated wrench
setp.input_double_register_6 = 0
setp.input_double_register_7 = 0
setp.input_double_register_8 = 0
setp.input_double_register_9 = 0
setp.input_double_register_10 = 0
setp.input_double_register_11 = 0

setp.input_bit_registers0_to_31 = 0

setp.input_bit_register_64 = False  # this boolean is used for the echo between the Robot and the external application
# this boolean is the check boolean in the polyscope - > in our logic we read the state of the output_bit_registers_65
# than we assign it to our setp.input_bit_registers_64, and we keep comparing both every 0.2 sec, so we know that the
# external application is running

watchdog.input_int_register_0 = 0  # represents the mode 1/ 2/ 3 (movej/ servoj/ halt())
# will not be used in this Application, since we are only interested in the impedance control


# start data synchronization
if not con.send_start():
    sys.exit()
# ------------------- stiffness/ Gains initialization ---------------------------------------
wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# ------------------- Points for the Trajectory/ Path initialization ------------------------

start_pose = [0.14605, 0.91942, 0.70804, 2.014, -2.072, -0.696]
desired_pose_1 = [-0.40662, 0.83772, 0.70590, 2.447, -1.321, -0.564]
desired_pose_2 = [-0.39653, 0.92225, 0.24794, 2.343, -1.354, -0.460]
poses = [start_pose, desired_pose_1, desired_pose_2, desired_pose_1, start_pose]

# ------------------- stiffness/ Gains initialization ---------------------------------------
# damping 0.0015
stiffness_tx = 3000  # stiffness for the translation along the x-Axis
stiffness_ty = 3000  # stiffness for the translation along the y-Axis
stiffness_tz = 3000  # stiffness for the translation along the z-Axis
stiffness_Rx = 50   # stiffness for the rotation about the x-Axis
stiffness_Ry = 50   # stiffness for the rotation about the y-Axis
stiffness_Rz = 50   # stiffness for the rotation about the z-Axis

damping_tx = 40  # damping for the translation along the x-Axis
damping_ty = 40  # damping for the translation along the y-Axis
damping_tz = 40  # damping for the translation along the z-Axis
damping_Rx = 1   # damping for the rotation about the x-Axis
damping_Ry = 1   # damping for the rotation about the y-Axis
damping_Rz = 1   # damping for the rotation about the z-Axis
damping = [damping_tx, damping_ty, damping_tz, damping_Rx, damping_Ry, damping_Rz]
damping = np.diag(damping)  # damping matrix
# ------------------ Control loop initialization ----------------------------------------------

state = con.receive()
list_to_setp_initcfg(setp, start_pose)  # changing initial pose to setp
list_to_setp_wrench(setp, wrench)
con.send(setp)
#   ------------  mode = 1 (Connection) -----------
print('Boolean 1 is False, please click CONTINUE on the Polyscope')
while True:
    state = con.receive()
    con.send(watchdog)
    # print(f"runtime state is {state.runtime_state}")
    if state.output_bit_registers0_to_31:
        print('Boolean 1 is True, Robot Program can proceed to mode 1\n')
        break

print("-------Executing moveJ -----------\n")

watchdog.input_int_register_0 = 1
con.send(watchdog)  # sending mode == 1
list_to_setp_initcfg(setp, start_pose)  # changing initial pose to setp
list_to_setp_wrench(setp, wrench)
con.send(setp)  # sending initial pose
print('Waiting for movej() to finish')
while True:
    state = con.receive()
    con.send(watchdog)
    if not state.output_bit_registers0_to_31:
        print()
        print('Initial configuration already reached, Proceeding to execute the Trajectory in impedance control mode\n')
        print()
        break

watchdog.input_int_register_0 = 2
con.send(watchdog)  # sending mode == 2

time_samples = 0.008  # 8 ms as a sampling time for the trajectory
trajectory_time = 4  # 4 sec for the entire trajectory

N = int(trajectory_time // time_samples)  # Number of samples, 0.008 sec sampling Time

plotter = True
# ------------------ Control loop initialization -------------------------
state = con.receive()
state_pose_scaled = pose_scal(state.actual_TCP_pose)

traj = get_Traj(poses, trajectory_time, time_samples)  # trajectory S->1
wrenchCalculation = WrenchForImpedance([stiffness_tx, stiffness_ty, stiffness_tz, stiffness_Rx, stiffness_Ry,
                                        stiffness_Rz])


# 4 sec final time of the Trajectory

if plotter:
    time_plot = []

    posx = []
    posy = []
    posz = []
    posRx = []
    posRy = []
    posRz = []

    F_x = []
    F_y = []
    F_z = []
    M_Rx = []
    M_Ry = []
    M_Rz = []

    current_x = []
    current_y = []
    current_z = []
    current_Rx = []
    current_Ry = []
    current_Rz = []

#   -------------------------Control loop --------------------
state = con.receive()
tcp = state.actual_TCP_pose
t_current = 0
t_prev = 0
i = 0
t_start = time.time()
while i < (len(poses) - 1) * N:
    state = con.receive()
    state_scaled = pose_scal(state.actual_TCP_pose)
    t_current = time.time() - t_start
    dt = t_current - t_prev
    if state.runtime_state > 1:
        Pose = get_pose(traj[i])
        current_pose = state_scaled
        current_speed = state.actual_TCP_speed
        wrench_k = np.array(wrenchCalculation.WrenchCalculation(Pose, current_pose))
        wrench_d = dampingWrench(current_speed, damping)
        wrench = wrench_k - wrench_d
        # wrench = wrench_k
        wrench = np.ravel(wrench).tolist()
        if plotter:
            time_plot.append(time.time() - t_start)

            posx.append(Pose[0])
            posy.append(Pose[1])
            posz.append(Pose[2])
            posRx.append(Pose[3])
            posRy.append(Pose[4])
            posRz.append(Pose[5])

            current_x.append(current_pose[0])
            current_y.append(current_pose[1])
            current_z.append(current_pose[2])
            current_Rx.append(current_pose[3])
            current_Ry.append(current_pose[4])
            current_Rz.append(current_pose[5])

            F_x.append(wrench[0])
            F_y.append(wrench[1])
            F_z.append(wrench[2])
            M_Rx.append(wrench[3])
            M_Ry.append(wrench[4])
            M_Rz.append(wrench[5])
        while dt < 0.005:
            t_current = time.time() - t_start
            dt = t_current - t_prev
        t_prev = t_current
        list_to_setp_wrench(setp, wrench)
        con.send(setp)
        i += 1
    else:
        break

print(f"It took {time.time() - t_start} sec  to finish")

con.send(setp)

state = con.receive()
print('--------------------\n')
print(pose_scal(state.actual_TCP_pose))

# ====================mode 3===================
watchdog.input_int_register_0 = 3
con.send(watchdog)

con.send_pause()
con.disconnect()

if plotter:
    # ----------- position -------------
    col1 = 'steelblue'
    col2 = 'red'
    col3 = 'green'

    fig, ax1 = plt.subplots()
    color = 'tab:blue'
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('X desired position', color=col1)
    ax1.plot(time_plot, posx, color=col1)
    ax1.tick_params(axis='y', labelcolor=col1)

    # ax1.set_ylabel('X Robot position', color=col2)
    ax1.plot(time_plot, current_x, color=col2)
    ax1.tick_params(axis='y', labelcolor=col2)
    # ax3 = ax1.twinx()
    # color = 'tab:green'
    # ax3.spines["right"].set_position(("axes", 1.2))
    # ax3.set_ylabel('X wrench', color=col3)
    # ax3.plot(time_plot, F_x, color=col3)
    # ax3.tick_params(axis='y', labelcolor=col3)
    fig.tight_layout()
    fig.savefig('Plots/plot_dist/x_wrench_position.svg', format='svg', dpi=1200)

    fig, ax1 = plt.subplots()
    color = 'tab:blue'
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('Y desired position', color=col1)
    ax1.plot(time_plot, posy, color=col1)
    ax1.tick_params(axis='y', labelcolor=col1)
    ax2 = ax1.twinx()
    color = 'tab:red'
    ax2.set_ylabel('Y Robot position', color=col2)
    ax2.plot(time_plot, current_y, color=col2)
    ax2.tick_params(axis='y', labelcolor=col2)
    # ax3 = ax1.twinx()
    # color = 'tab:green'
    # ax3.spines["right"].set_position(("axes", 1.2))
    # ax3.set_ylabel('Y wrench', color=col3)
    # ax3.plot(time_plot, F_y, color=col3)
    # ax3.tick_params(axis='y', labelcolor=col3)
    fig.tight_layout()
    fig.savefig('Plots/plot_dist/y_wrench_position.svg', format='svg', dpi=1200)

    fig, ax1 = plt.subplots()
    color = 'tab:blue'
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('z desired position', color=col1)
    ax1.plot(time_plot, posz, color=col1)
    ax1.tick_params(axis='y', labelcolor=col1)
    ax2 = ax1.twinx()
    color = 'tab:red'
    ax2.set_ylabel('Z Robot position', color=col2)
    ax2.plot(time_plot, current_z, color=col2)
    ax2.tick_params(axis='y', labelcolor=col2)
    # ax3 = ax1.twinx()
    # color = 'tab:green'
    # ax3.spines["right"].set_position(("axes", 1.2))
    # ax3.set_ylabel('Z wrench', color=col3)
    # ax3.plot(time_plot, F_z, color=col3)
    # ax3.tick_params(axis='y', labelcolor=col3)
    fig.tight_layout()
    fig.savefig('Plots/plot_dist/z_wrench_position.svg', format='svg', dpi=1200)

    fig, ax1 = plt.subplots()
    color = 'tab:blue'
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('x desired orientation', color=col1)
    ax1.plot(time_plot, posRx, color=col1)
    ax1.tick_params(axis='y', labelcolor=col1)
    ax2 = ax1.twinx()
    color = 'tab:red'
    ax2.set_ylabel('x Robot orientation', color=col2)
    ax2.plot(time_plot, current_Rx, color=col2)
    ax2.tick_params(axis='y', labelcolor=col2)
    # ax3 = ax1.twinx()
    # color = 'tab:green'
    # ax3.spines["right"].set_position(("axes", 1.2))
    # ax3.set_ylabel('x wrench', color=col3)
    # ax3.plot(time_plot, M_Rx, color=col3)
    # ax3.tick_params(axis='y', labelcolor=col3)
    fig.tight_layout()
    fig.savefig('Plots/plot_dist/x_wrench_orientation.svg', format='svg', dpi=1200)

    fig, ax1 = plt.subplots()
    color = 'tab:blue'
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('y desired orientation', color=col1)
    ax1.plot(time_plot, posRy, color=col1)
    ax1.tick_params(axis='y', labelcolor=col1)
    ax2 = ax1.twinx()
    color = 'tab:red'
    ax2.set_ylabel('y Robot orientation', color=col2)
    ax2.plot(time_plot, current_Ry, color=col2)
    ax2.tick_params(axis='y', labelcolor=col2)
    # ax3 = ax1.twinx()
    # color = 'tab:green'
    # ax3.spines["right"].set_position(("axes", 1.2))
    # ax3.set_ylabel('y wrench', color=col3)
    # ax3.plot(time_plot, M_Ry, color=col3)
    # ax3.tick_params(axis='y', labelcolor=col3)
    fig.tight_layout()
    fig.savefig('Plots/plot_dist/y_wrench_orientation.svg', format='svg', dpi=1200)

    fig, ax1 = plt.subplots()
    color = 'tab:blue'
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('z desired orientation', color=col1)
    ax1.plot(time_plot, posRz, color=col1)
    ax1.tick_params(axis='y', labelcolor=col1)
    ax2 = ax1.twinx()
    color = 'tab:red'
    ax2.set_ylabel('Z Robot orientation', color=col2)
    ax2.plot(time_plot, current_Rz, color=col2)
    ax2.tick_params(axis='y', labelcolor=col2)
    # ax3 = ax1.twinx()
    # color = 'tab:green'
    # ax3.spines["right"].set_position(("axes", 1.2))
    # ax3.set_ylabel('Z wrench', color=col3)
    # ax3.plot(time_plot, M_Rz, color=col3)
    # ax3.tick_params(axis='y', labelcolor=col3)
    fig.tight_layout()
    fig.savefig('Plots/plot_dist/z_wrench_orientation.svg', format='svg', dpi=1200)
    plt.show()
