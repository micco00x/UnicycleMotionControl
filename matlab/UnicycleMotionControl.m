% Clean workspace:
clc
clear
close all

% Simulation hparams:
sampling_interval = 0.01; % [s]
num_unicycles_to_draw = 20;

% Initial configuration of the unicycle:
unicycle_configuration = zeros(3, 1); % [m], [m], [rad]

% Desired trajectory:
circular_trajectory_center = [4.0; 4.0];
circular_trajectory_radius = 3.0;
circular_trajectory_desired_driving_velocity = 1.0;
circular_trajectory_phi = -pi / 2.0;
circular_trajectory_duration = 2.0 * pi * circular_trajectory_radius * circular_trajectory_desired_driving_velocity;
circular_trajectory = CircularTrajectory( ...
    circular_trajectory_center, ...
    circular_trajectory_radius, ...
    circular_trajectory_desired_driving_velocity, ...
    circular_trajectory_phi, ...
    circular_trajectory_duration ...
);

eight_shaped_trajectory_starting_pose = [4.0; 1.0; 1.0];
eight_shaped_trajectory_desired_steering_velocity = 0.6;
eight_shaped_trajectory = EightShapedTrajectory( ...
    eight_shaped_trajectory_starting_pose, ...
    eight_shaped_trajectory_desired_steering_velocity ...
);

desired_trajectory = eight_shaped_trajectory; % circular_trajectory, eight_shaped_trajectory

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
