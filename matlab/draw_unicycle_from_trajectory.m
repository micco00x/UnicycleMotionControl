% Draw the unicycle moving along a trajectory specified by vectors x, y,
% theta.
function  draw_unicycle_from_trajectory(x, y, theta)
    N = size(x);
    for k = 1:N
        unicycle_polygon = nsidedpoly(3, 'Center', [x(k), y(k)], 'SideLength', 0.3);
        unicycle_polygon = rotate(unicycle_polygon, 180 * theta(k) / pi, [x(k), y(k)]);
        plot(unicycle_polygon, 'FaceColor', 'white');
        hold on
    end
    
    plot(x, y, 'k-');
    daspect([1 1 1]);
    axis([-2 2 -2 2]);
    xlabel('[m]');
    ylabel('[m]');
end
