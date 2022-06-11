import math
import matplotlib.pyplot as plt
import numpy as np
import os

SAVE_FIGURES = True
SHOW_PLOTS = False

class UnicycleCommand:
    def __init__(self, driving_velocity, steering_velocity):
        self.driving_velocity = driving_velocity
        self.steering_velocity = steering_velocity

class UnicycleConfiguration:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

class UnicycleVelocity:
    def __init__(self, x_dot, y_dot, theta_dot):
        self.x_dot = x_dot
        self.y_dot = y_dot
        self.theta_dot = theta_dot

class Position2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y

if __name__ == '__main__':
    root_folder = "/tmp/UnicycleMotionControl"
    time_log_path = os.path.join(root_folder, "time_log.txt")
    unicycle_cmd_log_path = os.path.join(root_folder, "unicycle_cmd_log.txt")
    unicycle_configuration_log_path = os.path.join(root_folder, "unicycle_configuration_log.txt")
    unicycle_desired_pose_log_path = os.path.join(root_folder, "unicycle_desired_pose_log.txt")
    unicycle_measured_velocity_log_path = os.path.join(root_folder, "unicycle_measured_velocity_log.txt")
    unicycle_desired_velocity_log_path = os.path.join(root_folder, "unicycle_desired_velocity_log.txt")

    time_data = []
    command_data = []
    configuration_data = []
    desired_pose_data = []
    measured_velocity_data = []
    desired_velocity_data = []

    # Read time:
    with open(time_log_path) as time_log_file:
        for l in time_log_file.readlines():
            data = l.rstrip().split()
            t = float(data[0])
            time_data.append(t)

    # Read unicycle commands:
    with open(unicycle_cmd_log_path) as unicycle_cmd_log_file:
        for l in unicycle_cmd_log_file.readlines():
            data = l.rstrip().split()
            driving_velocity = float(data[0])
            steering_velocity = float(data[1])
            command_data.append(UnicycleCommand(driving_velocity, steering_velocity))
    
    # Read unicycle configurations:
    with open(unicycle_configuration_log_path) as unicycle_configuration_file:
        for l in unicycle_configuration_file.readlines():
            data = l.rstrip().split()
            x = float(data[0])
            y = float(data[1])
            theta = float(data[2])
            configuration_data.append(UnicycleConfiguration(x, y, theta))

    # Read unicycle desired poses:
    with open(unicycle_desired_pose_log_path) as unicycle_desired_pose_file:
        for l in unicycle_desired_pose_file.readlines():
            data = l.rstrip().split()
            x = float(data[0])
            y = float(data[1])
            theta = float(data[2])
            desired_pose_data.append(UnicycleConfiguration(x, y, theta))

    # Read unicycle measured velocities:
    with open(unicycle_measured_velocity_log_path) as unicycle_measured_velocity_file:
        for l in unicycle_measured_velocity_file.readlines():
            data = l.rstrip().split()
            x_dot = float(data[0])
            y_dot = float(data[1])
            theta_dot = float(data[2])
            measured_velocity_data.append(UnicycleVelocity(x_dot, y_dot, theta_dot))

    # Read unicycle desired velocities:
    with open(unicycle_desired_velocity_log_path) as unicycle_desired_velocity_file:
        for l in unicycle_desired_velocity_file.readlines():
            data = l.rstrip().split()
            x_dot = float(data[0])
            y_dot = float(data[1])
            theta_dot = float(data[2])
            desired_velocity_data.append(UnicycleVelocity(x_dot, y_dot, theta_dot))

    max_t = math.inf
    # Find id of element in time_data such that it is greater than max_t:
    idx_max_t = len(time_data)
    for idx, t in enumerate(time_data):
        if (t >= max_t):
            idx_max_t = idx
            break

    time_data = time_data[0:idx_max_t]
    command_data = command_data[0:idx_max_t]
    configuration_data = configuration_data[0:idx_max_t]
    desired_pose_data = desired_pose_data[0:idx_max_t]
    measured_velocity_data = measured_velocity_data[0:idx_max_t]
    desired_velocity_data = desired_velocity_data[0:idx_max_t]


    # Plot unicycle commands:
    legend_location = 'upper right'
    plt.rcParams['text.usetex'] = True

    s_label = r'$[\rm{s}]$'
    m_label = r'$[\rm{m}]$'
    rad_label = r'$[\rm{rad}]$'
    lv_label = r'$[\rm{m}/\rm{s}]$'
    rv_label = r'$[\rm{rad}/\rm{s}]$'

    fig_commands = plt.figure()

    ax_driving_velocity = fig_commands.add_subplot(2, 1, 1)
    #ax_driving_velocity.set_xlabel(s_label)
    ax_driving_velocity.set_ylabel(lv_label)
    ax_driving_velocity.plot(time_data, [cmd.driving_velocity for cmd in command_data], label=r'$v$')
    ax_driving_velocity.legend(loc=legend_location)
    ax_driving_velocity.grid()

    ax_steering_velocity = fig_commands.add_subplot(2, 1, 2)
    ax_steering_velocity.set_xlabel(s_label)
    ax_steering_velocity.set_ylabel(rv_label)
    ax_steering_velocity.plot(time_data, [cmd.steering_velocity for cmd in command_data], label=r'$\omega$')
    ax_steering_velocity.legend(loc=legend_location)
    ax_steering_velocity.grid()

    # Plot comparison between pose and desired pose:
    fig_positions = plt.figure()

    ax_pos_x = fig_positions.add_subplot(3, 1, 1)
    #ax_pos_x.set_xlabel(s_label)
    ax_pos_x.set_ylabel(m_label)
    ax_pos_x.plot(time_data, [q.x for q in configuration_data], label=r'$x$')
    ax_pos_x.plot(time_data, [q.x for q in desired_pose_data], label=r'$x_d$')
    ax_pos_x.legend(loc=legend_location)
    ax_pos_x.grid()

    ax_pos_y = fig_positions.add_subplot(3, 1, 2)
    #ax_pos_y.set_xlabel(s_label)
    ax_pos_y.set_ylabel(m_label)
    ax_pos_y.plot(time_data, [q.y for q in configuration_data], label=r'$y$')
    ax_pos_y.plot(time_data, [q.y for q in desired_pose_data], label=r'$y_d$')
    ax_pos_y.legend(loc=legend_location)
    ax_pos_y.grid()

    ax_pos_theta = fig_positions.add_subplot(3, 1, 3)
    ax_pos_theta.set_xlabel(s_label)
    ax_pos_theta.set_ylabel(rad_label)
    ax_pos_theta.plot(time_data, [q.theta for q in configuration_data], label=r'$\theta$')
    ax_pos_theta.plot(time_data, [q.theta for q in desired_pose_data], label=r'$\theta_d$')
    ax_pos_theta.legend(loc=legend_location)
    ax_pos_theta.grid()

    # Plot cartesian error:
    cartesian_error_data = []
    for qd, q in zip(desired_pose_data, configuration_data):
        err = math.sqrt(math.pow(qd.x - q.x, 2.0) + math.pow(qd.y - q.y, 2.0))
        cartesian_error_data.append(err)

    fig_cartesian_err = plt.figure()

    ax_cartesian_err = fig_cartesian_err.add_subplot(1, 1, 1)
    ax_cartesian_err.set_xlabel(s_label)
    ax_cartesian_err.set_ylabel(m_label)
    ax_cartesian_err.plot(time_data, cartesian_error_data, label=r'$e_p$')
    ax_cartesian_err.legend(loc=legend_location)
    ax_cartesian_err.grid()

    # Plot unicycle velocity:
    fig_velocities = plt.figure()

    ax_pos_x_dot = fig_velocities.add_subplot(3, 1, 1)
    #ax_pos_x_dot.set_xlabel(s_label)
    ax_pos_x_dot.set_ylabel(lv_label)
    ax_pos_x_dot.plot(time_data, [q_dot.x_dot for q_dot in measured_velocity_data], label=r'$\dot{x}$')
    ax_pos_x_dot.plot(time_data, [q_dot.x_dot for q_dot in desired_velocity_data], label=r'$\dot{x}_d$')
    ax_pos_x_dot.legend(loc=legend_location)
    ax_pos_x_dot.grid()

    ax_pos_y_dot = fig_velocities.add_subplot(3, 1, 2)
    #ax_pos_y_dot.set_xlabel(s_label)
    ax_pos_y_dot.set_ylabel(lv_label)
    ax_pos_y_dot.plot(time_data, [q_dot.y_dot for q_dot in measured_velocity_data], label=r'$\dot{y}$')
    ax_pos_y_dot.plot(time_data, [q_dot.y_dot for q_dot in desired_velocity_data], label=r'$\dot{y}_d$')
    ax_pos_y_dot.legend(loc=legend_location)
    ax_pos_y_dot.grid()

    ax_pos_theta_dot = fig_velocities.add_subplot(3, 1, 3)
    ax_pos_theta_dot.set_xlabel(s_label)
    ax_pos_theta_dot.set_ylabel(rv_label)
    ax_pos_theta_dot.plot(time_data, [q_dot.theta_dot for q_dot in measured_velocity_data], label=r'$\dot{\theta}$')
    ax_pos_theta_dot.plot(time_data, [q_dot.theta_dot for q_dot in desired_velocity_data], label=r'$\dot{\theta}_d$')
    ax_pos_theta_dot.legend(loc=legend_location)
    ax_pos_theta_dot.grid()

    # Show plots:
    if SHOW_PLOTS:
        plt.show()

    # Save figures:
    if SAVE_FIGURES:
        dpi = 300.0 # standard DPI for printing use
        fig_commands.savefig(os.path.join(root_folder, 'unicycle_commands.png'), dpi=dpi)
        fig_positions.savefig(os.path.join(root_folder, 'positions.png'), dpi=dpi)
        fig_cartesian_err.savefig(os.path.join(root_folder, 'cartesian_error.png'), dpi=dpi)
        fig_velocities.savefig(os.path.join(root_folder, 'velocities.png'), dpi=dpi)