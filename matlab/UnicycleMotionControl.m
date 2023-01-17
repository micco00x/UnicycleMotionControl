% Clean workspace:
clc
clear
close all

% Trajectory of the unicycle:
N = 100;

unicycle_configurations = [
    linspace(0.0, 10.0, N)', ...
    zeros(N, 1), ...
    zeros(N, 1)
];

% Draw unicycle given the trajectory:
draw_unicycle_from_trajectory(unicycle_configurations, 10, 'black');
