% Clean workspace:
clc
clear
close all

% Simulation hparams:
sampling_interval = 0.01; % [s]
num_unicycles_to_draw = 20;

% Initial configuration of the unicycle:
unicycle_configuration = [1.0; 0.0; 0.0]; % [m], [m], [rad]
unicycle_velocity = zeros(3, 1);

% Circular trajectory hparams:
circular_trajectory_center = [4.0; 4.0]; % [m]
circular_trajectory_radius = 3.0; % [m]
circular_trajectory_desired_driving_velocity = 1.0; % [m/s]
circular_trajectory_phi = -pi / 2.0; % [rad]
circular_trajectory_duration = 2.0 * pi * circular_trajectory_radius / circular_trajectory_desired_driving_velocity; % [s]
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

% Approximate linearization controller hparams:
approximate_linearization_controller_zeta = 0.7;
approximate_linearization_controller_a = 2.0;
approximate_linearization_controller = ApproximateLinearizationController( ...
    approximate_linearization_controller_zeta, ...
    approximate_linearization_controller_a ...
);

% Dynamic feedback linearization controller hparams:
dynamic_feedback_linearization_controller_kp1 = 4.0;
dynamic_feedback_linearization_controller_kp2 = 4.0;
dynamic_feedback_linearization_controller_kd1 = 4.0;
dynamic_feedback_linearization_controller_kd2 = 4.0;
dynamic_feedback_linearization_controller_xi_0 = 1.0;
dynamic_feedback_linearization_controller = DynamicFeedbackLinearizationController( ...
    dynamic_feedback_linearization_controller_kp1, ...
    dynamic_feedback_linearization_controller_kp2, ...
    dynamic_feedback_linearization_controller_kd1, ...
    dynamic_feedback_linearization_controller_kd2, ...
    sampling_interval, ...
    dynamic_feedback_linearization_controller_xi_0 ...
);

% Static feedback linearization controller hparams:
static_feedback_linearization_controller_b = 0.75;
static_feedback_linearization_controller_k1 = 2.0;
static_feedback_linearization_controller_k2 = 2.0;
static_feedback_linearization_controller = StaticFeedbackLinearizationController( ...
    static_feedback_linearization_controller_b, ...
    static_feedback_linearization_controller_k1, ...
    static_feedback_linearization_controller_k2 ...
);

% Desired trajectory and controller:
desired_trajectory_type = TrajectoryType.Squared; % Circular, EightShaped, Squared
controller_type = ControllerType.StaticFeedbackLinearization; % ApproximateLinearization, DynamicFeedbackLinearization, StaticFeedbackLinearization

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% DO NOT MODIFY ANYTHING BELOW UNLESS YOU KNOW WHAT YOU ARE DOING %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

switch desired_trajectory_type
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

switch controller_type
    case ControllerType.ApproximateLinearization
        trajectory_tracking_controller = approximate_linearization_controller;
    case ControllerType.DynamicFeedbackLinearization
        trajectory_tracking_controller = dynamic_feedback_linearization_controller;
    case ControllerType.StaticFeedbackLinearization
        trajectory_tracking_controller = static_feedback_linearization_controller;
    otherwise
        disp('Controller must be of the type ApproximateLinearization, DynamicFeedbackLinearization or StaticFeedbackLinearization.');
        return;
end

iterations = fix(desired_trajectory.duration / sampling_interval);

% Variables to be used for plotting:
time = linspace(0.0, iterations * sampling_interval, iterations);
unicycle_configuration_log = zeros(iterations, 3);
unicycle_configuration_ref_log = zeros(iterations, 3);
unicycle_velocity_log = zeros(iterations, 3);
unicycle_velocity_ref_log = zeros(iterations, 3);
control_input_log = zeros(iterations, 2);
tracking_error_log = zeros(iterations, 1);

% Run simulation:
for iter = 1:iterations
    % Compute commands using selected controller:
    control_input = trajectory_tracking_controller.compute_commands(time(iter), unicycle_configuration, unicycle_velocity, desired_trajectory);

    % Log:
    [unicycle_configuration_ref, unicycle_velocity_ref, ~] = desired_trajectory.eval(time(iter));
    unicycle_configuration_log(iter, :) = unicycle_configuration;
    unicycle_configuration_ref_log(iter, :) = unicycle_configuration_ref;
    unicycle_velocity_log(iter, :) = unicycle_velocity;
    unicycle_velocity_ref_log(iter, :) = unicycle_velocity_ref;
    control_input_log(iter, :) = control_input;
    tracking_error_log(iter, 1) = norm(unicycle_configuration_ref(1:2) - unicycle_configuration(1:2));

    % Simulation step:
    [unicycle_configuration, unicycle_velocity] = simulate_unicycle_motion(unicycle_configuration, control_input, sampling_interval);

end

% Draw unicycle given the trajectory:
figure
draw_unicycle_from_trajectory(unicycle_configuration_log, num_unicycles_to_draw, 'black');
hold on
%draw_unicycle_from_trajectory(unicycle_configuration_ref_log, num_unicycles_to_draw, 'green');
plot(unicycle_configuration_ref_log(:, 1), unicycle_configuration_ref_log(:, 2), 'Color', 'black', 'LineStyle', ':');

% Plot commands:
figure;
subplot(2, 1, 1);
plot(time, control_input_log(:, 1));
grid on;
title('driving velocity');
xlabel('[s]');
ylabel('[m/s]');
subplot(2, 1, 2);
plot(time, control_input_log(:, 2));
grid on;
title('steering velocity');
xlabel('[s]');
ylabel('[rad/s]');

% Plot tracking error:
figure;
plot(time, tracking_error_log);
grid on;
title('tracking error norm');
xlabel('[s]');
ylabel('[m]');

% Plot unicycle configurations:
figure;
subplot(3, 1, 1);
plot(time, unicycle_configuration_log(:, 1), time, unicycle_configuration_ref_log(:, 1));
grid on;
xlabel('[s]');
ylabel('[m]');
legend('x', 'x ref');
subplot(3, 1, 2);
plot(time, unicycle_configuration_log(:, 2), time, unicycle_configuration_ref_log(:, 2));
grid on;
xlabel('[s]');
ylabel('[m]');
legend('y', 'y ref');
subplot(3, 1, 3);
plot(time, unicycle_configuration_log(:, 3), time, unicycle_configuration_ref_log(:, 3));
grid on;
xlabel('[s]');
ylabel('[rad]');
legend('\theta', '\theta ref');

% Plot unicycle velocities:
figure;
subplot(3, 1, 1);
plot(time, unicycle_velocity_log(:, 1), time, unicycle_velocity_ref_log(:, 1));
grid on;
xlabel('[s]');
ylabel('[m/s]');
legend('x dot', 'x dot ref');
subplot(3, 1, 2);
plot(time, unicycle_velocity_log(:, 2), time, unicycle_velocity_ref_log(:, 2));
grid on;
xlabel('[s]');
ylabel('[m/s]');
legend('y dot', 'y dot ref');
subplot(3, 1, 3);
plot(time, unicycle_velocity_log(:, 3), time, unicycle_velocity_ref_log(:, 3));
grid on;
xlabel('[s]');
ylabel('[rad/s]');
legend('\theta dot', '\theta dot ref');
