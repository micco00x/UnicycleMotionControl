import matplotlib.pyplot as plt
import os

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
    unicycle_desired_velocity_log_path = os.path.join(root_folder, "unicycle_desired_velocity_log.txt")

    time_data = []
    command_data = []
    configuration_data = []
    desired_pose_data = []
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

    # Read unicycle desired velocities:
    with open(unicycle_desired_velocity_log_path) as unicycle_desired_velocity_file:
        for l in unicycle_desired_velocity_file.readlines():
            data = l.rstrip().split()
            x_dot = float(data[0])
            y_dot = float(data[1])
            theta_dot = float(data[2])
            desired_velocity_data.append(UnicycleVelocity(x_dot, y_dot, theta_dot))


    # Plot unicycle commands:
    fig_commands = plt.figure()

    ax_driving_velocity = fig_commands.add_subplot(2, 1, 1)
    ax_driving_velocity.set_xlabel('[s]')
    ax_driving_velocity.set_ylabel('[m/s]')
    ax_driving_velocity.plot(time_data, [cmd.driving_velocity for cmd in command_data], label='driving velocity')
    ax_driving_velocity.legend()
    ax_driving_velocity.grid()

    ax_steering_velocity = fig_commands.add_subplot(2, 1, 2)
    ax_steering_velocity.set_xlabel('[s]')
    ax_steering_velocity.set_ylabel('[rad/s]')
    ax_steering_velocity.plot(time_data, [cmd.steering_velocity for cmd in command_data], label='steering velocity')
    ax_steering_velocity.legend()
    ax_steering_velocity.grid()

    # Plot comparison between pose and desired pose:
    fig_positions = plt.figure()

    ax_pos_x = fig_positions.add_subplot(3, 1, 1)
    ax_pos_x.set_xlabel('[s]')
    ax_pos_x.set_ylabel('[m]')
    ax_pos_x.plot(time_data, [q.x for q in configuration_data], label='x')
    ax_pos_x.plot(time_data, [q.x for q in desired_pose_data], label='x des')
    ax_pos_x.legend()
    ax_pos_x.grid()

    ax_pos_y = fig_positions.add_subplot(3, 1, 2)
    ax_pos_y.set_xlabel('[s]')
    ax_pos_y.set_ylabel('[m]')
    ax_pos_y.plot(time_data, [q.y for q in configuration_data], label='y')
    ax_pos_y.plot(time_data, [q.y for q in desired_pose_data], label='y des')
    ax_pos_y.legend()
    ax_pos_y.grid()

    ax_pos_theta = fig_positions.add_subplot(3, 1, 3)
    ax_pos_theta.set_xlabel('[s]')
    ax_pos_theta.set_ylabel('[rad]')
    ax_pos_theta.plot(time_data, [q.theta for q in configuration_data], label='theta')
    ax_pos_theta.plot(time_data, [q.theta for q in desired_pose_data], label='theta des')
    ax_pos_theta.legend()
    ax_pos_theta.grid()

    # Plot unicycle velocity:
    fig_velocities = plt.figure()

    ax_pos_x_dot = fig_velocities.add_subplot(3, 1, 1)
    ax_pos_x_dot.set_xlabel('[s]')
    ax_pos_x_dot.set_ylabel('[m/s]')
    ax_pos_x_dot.plot(time_data, [q_dot.x_dot for q_dot in desired_velocity_data], label='x_dot des')
    ax_pos_x_dot.legend()
    ax_pos_x_dot.grid()

    ax_pos_y_dot = fig_velocities.add_subplot(3, 1, 2)
    ax_pos_y_dot.set_xlabel('[s]')
    ax_pos_y_dot.set_ylabel('[m/s]')
    ax_pos_y_dot.plot(time_data, [q_dot.y_dot for q_dot in desired_velocity_data], label='y_dot des')
    ax_pos_y_dot.legend()
    ax_pos_y_dot.grid()

    ax_pos_theta_dot = fig_velocities.add_subplot(3, 1, 3)
    ax_pos_theta_dot.set_xlabel('[s]')
    ax_pos_theta_dot.set_ylabel('[rad/s]')
    ax_pos_theta_dot.plot(time_data, [q_dot.theta_dot for q_dot in desired_velocity_data], label='theta_dot des')
    ax_pos_theta_dot.legend()
    ax_pos_theta_dot.grid()

    # Show plots:
    plt.show()