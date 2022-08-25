import csv
import math
import sys
import time

from examples.Automated_test import test_force_x_range, test_force_y_range, \
    test_vel_x_range, test_vel_y_range, test_impedance_x_range, test_impedance_y_range, test_torque_x_range, \
    test_torque_y_range, test_resultant_force_range, test_resultant_torque_range
from examples.DataBase_Automatedtesting import end_test_force_x_range, end_test_force_y_range, end_test_vel_x_range, \
    end_test_vel_y_range, end_test_torque_x_range, end_test_torque_y_range, end_test_resultant_force_range, \
    end_test_resultant_torque_range
from examples.plotting_Sensors import plot_TCP_force_x_time, \
    plot_velocity_x_time, plot_TCP_force_y_time, plot_velocity_y_time, plot_Impedance_x_time, plot_Impedance_y_time, \
    plot_torque_x_time, plot_torque_y_time, plot_TCP_pose_x_time, plot_TCP_pose_y_time, plot_resultant_force_time, \
    plot_resultant_torque_time

sys.path.append('..')
import logging
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

#logging.basicConfig(level=logging.INFO)

ROBOT_HOST = '134.28.124.86'
ROBOT_PORT = 30004
config_filename = 'control_loop_configuration.xml'

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe('state')
setp_names, setp_types = conf.get_recipe('setp')
watchdog_names, watchdog_types = conf.get_recipe('watchdog')

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# get controller version
con.get_controller_version()

# setup recipes
con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

keep_running = True
program_time = 27 #middle
# program_time = 80
y = []
t = []
tcp_pose = []
tcp_speed = []
tcp_position_x = []
tcp_position_y = []
tcp_velocity_x = []
tcp_velocity_y = []
tcp_force = []
tcp_force_x = []
tcp_force_y = []
tcp_pose_x = []
execution_time = []
time_stamp = []
impedance_x = []
impedance_y = []
torque_x = []
torque_y = []
resultant_force = []
resultant_torque = []

velocity_time_x = []
velocity_time_y = []
force_time_x = []
force_time_y = []
impedance_time_x = []
impedance_time_y = []
torque_time_x = []
torque_time_y = []
resultant_force_time = []
resultant_torque_time = []

# Setpoints to move the robot to
# setp1 = [0.05030, -0.47243, 0.18788, 0.051, -0.067, 2.270]
# setp2 = [0.05030, -0.88586, 0.18788, 0.051, -0.067, 2.270]
# setp1 = [-0.81255, 0.04050, 0.52630, 2.263, -0.956, 2.257] #good_1
# setp2 = [-1.16290, 0.04050, 0.52630, 2.263, -0.956, 2.257] #good_2
# setp_via = [-1.04680, -0.26345, 0.52629, 2.263, -0.956, 2.257] #good_via

# setp1 = [0.300, -0.66496, 0.24303, 1.101, -3.063, -0.190] #working well
# setp_via = [0.29377, -0.63351, 0.22478, 0.440, -3.136, 0.003]
# setp2 = [0.33540, -0.65187, 0.23965, 1.056, 2.887, -0.232]

# setp1 = [0.34404, -0.70171, 0.23919, 1.532, -2.866, -0.092] #working well infront of Alireza
# setp_via = [0.31650, -0.59194, 0.23536, 0.219, 3.101, 0.137]
# setp2 = [0.38004, -0.62235, 0.23790, 2.017, 2.277, -0.036]

# setp1 = [0.32649, -0.69103, 0.23765, 1.643, -2.775, 0.012] #new pose after fixing the hardware but sensor mounted so not accurate
# setp_via = [0.35527, -0.59448, 0.43890, 1.841, -0.790, -2.081]
# setp2 = [0.37651, -0.59854, 0.44501, 1.124, -2.677, -2.635]

# setp1 = [0.300, -0.66496, 0.24303, 1.101, -3.063, -0.190]
# setp_via = [0.29377, -0.63351, 0.22478, 0.440, -3.136, 0.003] #1st_WR
# setp2 = [0.33540, -0.65187, 0.23965, 1.056, 2.887, -0.232]

# setp1 = [0.33648, -0.66963, 0.24008, 2.605, -1.954, 0.037]
# setp_via = [0.23160, -0.55880, 0.23177, 0.470, -3.142, 0.027] #1st_WR
# setp2 = [0.35197, -0.47385, 0.24108, 1.992, 2.477, -0.110]

setp1 = [0.35846, -0.61026, 0.23813, 2.502, -2.097, 0.147] #FINAL STABLE
setp_via = [0.27490, -0.53090, 0.23413, 0.596, -3.096, 0.158]
setp2 = [0.37827, -0.42166, 0.23999, 1.766, 2.649, -0.195]

# vel1 = [0.05] #test medium in 1s
# acc1 = [0.05]
# vel1 = [1] #test quick in 0.8s
# acc1 = [1.2]
# vel1 = [0.01] #usedd test slow constant speed 0.01 in 0.2s
# acc1 = [0.05]
# vel1 = [1] #test slower : max.speed 0.01 in 2s
# acc1 = [0.0005]
# vel1 = [0.02] #test quick max.velocity it can get with 0.05 acc. in 3s
# acc1 = [0.05]

vel1 = [0.01] #usedd test slow constant speed 0.01 in 0.2s
acc1 = [0.0005]
# vel1 = [0.01] #usedd test very slow constant speed 0.01 in 0.2s
# acc1 = [0.00005]
# vel1 = [0.02] #usedd faster,not stable with WR
# acc1 = [0.05]

setp.input_double_register_0 = 0
setp.input_double_register_1 = 0
setp.input_double_register_2 = 0
setp.input_double_register_3 = 0
setp.input_double_register_4 = 0
setp.input_double_register_5 = 0
setp.input_double_register_6 = 0
setp.input_double_register_7 = 0

# The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
watchdog.input_int_register_0 = 0

def setp_to_list(setp):
    list = []
    for i in range(0,6):
        list.append(setp.__dict__["input_double_register_%i" % i])
    return list

#Convert position list to setPoints for the urp script to assign to every register
def pos_list_to_setp(setp, list):
    for i in range (0,6):
        setp.__dict__["input_double_register_%i" % i] = list[i]
    return setp

#Covert velocity list to setPoints for the urp script to assign to assign to register 6
def vel_list_to_setp(setp, list):
    for i in range(0, 1):
        count = i + 6
        setp.__dict__["input_double_register_%i" % count] = list[i]
        #count = count+1
    return setp

#Covert acceleration list to setPoints for the urp script to assign to register 7
def acc_list_to_setp(setp, list):
    for i in range(0, 1):
        count = i + 7
        setp.__dict__["input_double_register_%i" % count] = list[i]
        #count = count + 1
    return setp

#Covert acceleration list to setPoints for the urp script to assign to register 7
def pos_via_list_to_setp(setp, list):
    for i in range(0, 6):
        count = i + 8
        setp.__dict__["input_double_register_%i" % count] = list[i]
        #count = count + 1
    return setp

def distance_from_start_x(setp,state):

    distance = setp.input_double_register_0-state.actual_TCP_pose[0] #modify when using other robot
    #print(distance)
    return distance

#start data synchronization
if not con.send_start():
    sys.exit()

def export_tcp_pose_x_to_csv(list):

    file_to_output = open('TCP_pose_x.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_tcp_pose_y_to_csv(list):

    file_to_output = open('TCP_pose_y.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_tcp_speed_to_csv(list):

    file_to_output = open('TCP_speed.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_tcp_force_x_to_csv(list):

    file_to_output = open('TCP_force_x.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_tcp_force_y_to_csv(list):

    file_to_output = open('TCP_force_y.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_tcp_velocity_x_to_csv(list):

    file_to_output = open('TCP_velocity_x.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_tcp_velocity_y_to_csv(list):

    file_to_output = open('TCP_velocity_y.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_execution_time_to_csv(list):

    file_to_output = open('Execution_time.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_impedance_forcex_to_csv(list):
    file_to_output = open('impedance_x.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_impedance_forcey_to_csv(list):
    file_to_output = open('impedance_y.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_torque_x_to_csv(list):
    file_to_output = open('torque_x.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_torque_y_to_csv(list):
    file_to_output = open('torque_y.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_resultant_force_to_csv(list):
    file_to_output = open('resultant_force.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_resultant_torque_to_csv(list):
    file_to_output = open('resultant_torque.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def export_time_to_csv(list):
    file_to_output = open('time.csv', 'w', newline='')
    csv_writer = csv.writer(file_to_output, delimiter=',')
    csv_writer.writerow(list)

def loop_rtde():

    current_time = 0
    while keep_running and current_time <= program_time:
        # receive the current state
        state = con.receive()
        t2 = time.time()
        current_time = t2 - start_time
        print(f"The current time is  {current_time}")
        if state is None:
            break

        # do something...
        if state.output_int_register_0 == 0:
            if state.output_int_register_5 == 1:
                new_setp = setp1
                # print("pose1")
            if state.output_int_register_5 == 0:
                new_setp = setp2
                # print("pose2")
            pos_list_to_setp(setp, new_setp)
            pos_via_list_to_setp(setp, setp_via)
            vel_list_to_setp(setp, vel1)
            acc_list_to_setp(setp, acc1)

            ####Position x-direction####
            tcp_position_x.append(state.actual_TCP_pose[0])
            export_tcp_pose_x_to_csv(tcp_position_x)

            ####Position y-direction####
            tcp_position_y.append(state.actual_TCP_pose[1])
            export_tcp_pose_y_to_csv(tcp_position_y)

            ####Velocity x-direction####
            velocity_x = state.actual_TCP_speed[0]
            if test_vel_x_range(velocity_x):
                current_time_velocity_x = [test_vel_x_range(velocity_x), current_time]
                velocity_time_x.append(current_time_velocity_x)
            test_vel_x_range(velocity_x)  ##Print values not in range in real-time
            tcp_velocity_x.append(state.actual_TCP_speed[0])
            export_tcp_velocity_x_to_csv(tcp_velocity_x)

            ####Velocity y-direction####
            velocity_y = state.actual_TCP_speed[1]
            if test_vel_y_range(velocity_y):
                current_time_velocity_y = [test_vel_y_range(velocity_y), current_time]
                velocity_time_y.append(current_time_velocity_y)
            test_vel_y_range(velocity_y)  ##Print values not in range in real-time
            tcp_velocity_y.append(state.actual_TCP_speed[1])
            export_tcp_velocity_y_to_csv(tcp_velocity_y)

            ####Force x-direction####
            force_x = state.actual_TCP_force[0]
            if test_force_x_range(force_x): ##if not in the range put it in 2D list
                current_time_force_x = [test_force_x_range(force_x),current_time] ##put in a changing variable 2D-list with forces and corresponding time
                force_time_x.append(current_time_force_x) ## 2D-list with forces and corresponding time
            test_force_x_range(force_x) ##Print values not in range
            tcp_force_x.append(state.actual_TCP_force[0]) ##append in a list for export
            export_tcp_force_x_to_csv(tcp_force_x) ##export list to csv file

            ####Force y-direction####
            force_y = state.actual_TCP_force[1]
            if test_force_y_range(force_y):
                current_time_force_y = [test_force_y_range(force_y),current_time]
                force_time_y.append(current_time_force_y)
            test_force_y_range(force_y) ##Print values not in range in real-time
            tcp_force_y.append(state.actual_TCP_force[1])
            export_tcp_force_y_to_csv(tcp_force_y)
            # print(f"The actual TCP Force is {state.actual_TCP_force[0]}")

            #### Resultant force ####
            fx = pow(state.actual_TCP_force[0], 2)
            fy = pow(state.actual_TCP_force[1], 2)
            sum_force = fx + fy
            result_force = math.sqrt(sum_force)
            if test_resultant_force_range(result_force):
                current_time_resultant_force = [test_resultant_force_range(result_force), current_time]
                resultant_force_time.append(current_time_resultant_force)
            test_resultant_force_range(result_force)  ##Print values not in range in real-time
            resultant_force.append(result_force)
            export_resultant_force_to_csv(resultant_force)

            #### torque x-direction ####
            torqx = state.actual_TCP_force[0] / 8.1
            if test_torque_x_range(torqx):
                current_time_torque_x = [test_torque_x_range(torqx), current_time]
                torque_time_x.append(current_time_torque_x)
            test_torque_x_range(torqx)  ##Print values not in range in real-time
            torque_x.append(state.actual_TCP_force[3])
            export_torque_x_to_csv(torque_x)

            #### torque y-direction ####
            torqy = state.actual_TCP_force[1] / 8.1
            if test_torque_y_range(torqy):
                current_time_torque_y = [test_torque_y_range(torqy), current_time]
                torque_time_y.append(current_time_torque_y)
            test_torque_y_range(torqy)  ##Print values not in range in real-time
            torque_y.append(state.actual_TCP_force[4])
            export_torque_y_to_csv(torque_y)

            #### Resultant Torque ####
            tx = pow(state.actual_TCP_force[3], 2)
            ty = pow(state.actual_TCP_force[4], 2)
            sum_torque = tx + ty
            result_torque = math.sqrt(sum_torque)
            if test_resultant_torque_range(result_torque):
                current_time_resultant_torque = [test_resultant_torque_range(result_torque), current_time]
                resultant_torque_time.append(current_time_resultant_torque)
            test_resultant_torque_range(result_torque)  ##Print values not in range in real-time
            resultant_torque.append(result_torque)
            export_resultant_torque_to_csv(resultant_torque)

            ####Impedance X-direction####
            if state.actual_TCP_speed[0] == 0:
                impedancex = state.actual_TCP_force[0]
                # if test_impedance_x_range(impedancex):
                #     current_time_imp_x = [test_impedance_x_range(impedancex), current_time]
                #     impedance_time_x.append(current_time_imp_x)
                test_impedance_x_range(impedancex)  ##Print values not in range in real-time
                impedance_x.append(state.actual_TCP_force[0] / 100)
                export_impedance_forcex_to_csv(impedance_x)
            else:
                impedancex = state.actual_TCP_force[0] / state.actual_TCP_speed[0]
                # if test_impedance_x_range(impedancex):
                #     current_time_imp_x = [test_impedance_x_range(impedancex), current_time]
                #     impedance_time_x.append(current_time_imp_x)
                test_impedance_x_range(impedancex)
                impedance_x.append(state.actual_TCP_force[0] / 0.8 * state.actual_TCP_speed[0])
                export_impedance_forcex_to_csv(impedance_x)

            ####Impedance Y-direction####
            if state.actual_TCP_speed[1] == 0:
                impedancey = state.actual_TCP_force[1]
                # if test_impedance_y_range(impedancey):
                #     current_time_imp_y = [test_impedance_y_range(impedancey), current_time]
                #     impedance_time_y.append(current_time_imp_y)
                test_impedance_y_range(impedancey)
                impedance_y.append(state.actual_TCP_force[1] / 100 )
                export_impedance_forcey_to_csv(impedance_y)
            else:
                impedancey = state.actual_TCP_force[1] / state.actual_TCP_speed[1]
                # if test_impedance_y_range(impedancey):
                #     current_time_imp_y = [test_impedance_y_range(impedancey), current_time]
                #     impedance_time_y.append(current_time_imp_y)
                test_impedance_y_range(impedancey)  ##Print values not in range in real-time
                impedance_y.append(state.actual_TCP_force[1] / 0.8 * state.actual_TCP_speed[1])
                export_impedance_forcey_to_csv(impedance_y)

            #### time ####
            execution_time.append(current_time)
            export_execution_time_to_csv(execution_time)
            # print(f"The execution time is {current_time}")

            con.send(setp)

            if state.output_int_register_6 == 1:
                print("The distance is reached")

        # kick watchdog
        con.send(watchdog)


if __name__ == "__main__":

    start_time = time.time()
    loop_rtde()

    ### Plot ###
    plot_TCP_pose_x_time()
    plot_TCP_pose_y_time()
    plot_TCP_force_x_time()
    plot_TCP_force_y_time()
    plot_torque_x_time()
    plot_torque_y_time()
    plot_resultant_force_time()
    plot_resultant_torque_time()
    plot_velocity_x_time()
    plot_velocity_y_time()
    plot_Impedance_x_time()
    plot_Impedance_y_time()

    ### display data not in the range at the specified time ###
    end_test_force_x_range(force_time_x)
    end_test_force_y_range(force_time_y)
    end_test_vel_x_range(velocity_time_x)
    end_test_vel_y_range(velocity_time_y)
    end_test_torque_x_range(torque_time_x)
    end_test_torque_y_range(torque_time_y)
    end_test_resultant_force_range(resultant_force_time)
    end_test_resultant_torque_range(resultant_torque_time)

    print("Program finished")

# con.send_pause()
# con.disconnect()



