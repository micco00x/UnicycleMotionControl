% Clean workspace:
clc
clear
close all

% Simulation hparams:
sampling_interval = 0.01; % [s]
num_unicycles_to_draw = 20;

% Initial configuration of the unicycle:
unicycle_configuration = zeros(3, 1); % [m], [m], [rad]

% Circular trajectory hparams:
circular_trajectory_center = [4.0; 4.0]; % [m]
circular_trajectory_radius = 3.0; % [m]
circular_trajectory_desired_driving_velocity = 1.0; % [m/s]
circular_trajectory_phi = -pi / 2.0; % [rad]
circular_trajectory_duration = 2.0 * pi * circular_trajectory_radius * circular_trajectory_desired_driving_velocity; % [s]
circular_trajectory = CircularTrajectory( ...
    circular_trajectory_center, ...
    circular_trajectory_radius, ...
    circular_trajectory_desired_driving_velocity, ...
    circular_trajectory_phi, ...
    circular_trajectory_duration ...
);

% Eight shaped trajectory hparams:
eight_shaped_trajectory_starting_pose = [4.0; 1.0; 1.0]; % [m], [m], [rad]
eight_shaped_trajectory_desired_steering_velocity = 0.6; % [rad/s]
eight_shaped_trajectory = EightShapedTrajectory( ...
    eight_shaped_trajectory_starting_pose, ...
    eight_shaped_trajectory_desired_steering_velocity ...
);

% Squared trajectory hparams:
squared_trajectory_t0 = 0.0; % [s]
squared_trajectory_square_frame = [1.0; 0.5; 0.0]; % [m], [m], [rad]
squared_trajectory_desired_driving_velocity = 1.0; % [m/s]
squared_trajectory_square_length = 5.0; % [m]
squared_trajectory = SquaredTrajectory( ...
    squared_trajectory_t0, ...
    squared_trajectory_square_frame, ...
    squared_trajectory_desired_driving_velocity, ...
    squared_trajectory_square_length ...
);

% Trajectory:
trajectory_type = TrajectoryType.Circular; % Circular, EightShaped, Squared

switch trajectory_type
    case TrajectoryType.Circular
        desired_trajectory = circular_trajectory;
    case TrajectoryType.EightShaped
        desired_trajectory = eight_shaped_trajectory;
    case TrajectoryType.Squared
        desired_trajectory = squared_trajectory;
    otherwise
        disp('Trajectory must be of the type Circular, EightShaped or Squared.')
        return;
end

iterations = fix(desired_trajectory.duration / sampling_interval);

% Commands to be applied to the unicycle:
control_input = [1.0; 0.2]; % [m/s], [rad/s]

% Variables to be used for plotting:
time = linspace(0.0, iterations * sampling_interval, iterations);
unicycle_configuration_log = zeros(iterations, 3);
unicycle_configuration_ref_log = zeros(iterations, 3);

% Run simulation:
for iter = 1:iterations
    % Simulation step:
    unicycle_configuration = simulate_unicycle_motion(unicycle_configuration, control_input, sampling_interval);

    [unicycle_configuration_ref, ~, ~] = desired_trajectory.eval(time(iter));

    % Log:
    unicycle_configuration_log(iter, :) = unicycle_configuration;
    unicycle_configuration_ref_log(iter, :) = unicycle_configuration_ref;
end

% Draw unicycle given the trajectory:
draw_unicycle_from_trajectory(unicycle_configuration_log, num_unicycles_to_draw, 'black');
hold on
draw_unicycle_from_trajectory(unicycle_configuration_ref_log, num_unicycles_to_draw, 'green');
