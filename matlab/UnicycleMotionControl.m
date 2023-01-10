% Trajectory of the unicycle:
N = 6;
x = zeros(N, 1);
y = linspace(0.0, 1.0, N);
theta = zeros(N, 1);

% Draw unicycle given the trajectory:
draw_unicycle_from_trajectory(x, y, theta);
