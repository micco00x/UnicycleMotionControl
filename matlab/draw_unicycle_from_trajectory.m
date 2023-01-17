function  draw_unicycle_from_trajectory(unicycle_configurations, num_unicycles_to_draw, color)
    x = unicycle_configurations(:, 1);
    y = unicycle_configurations(:, 2);
    theta = unicycle_configurations(:, 3);
    N = size(x, 1);
    iters_to_draw = fix(linspace(1, N, num_unicycles_to_draw));
    for k = iters_to_draw
        unicycle_polygon = nsidedpoly(3, 'Center', [x(k), y(k)], 'SideLength', 0.3);
        unicycle_polygon = rotate(unicycle_polygon, 180 * (theta(k) - pi/2) / pi, [x(k), y(k)]);
        plot(unicycle_polygon, 'EdgeColor', color, 'FaceColor', 'white', 'HandleVisibility', 'off');
        hold on
    end
    
    % Add trajectory:
    plot(x(iters_to_draw), y(iters_to_draw), 'Color', color, 'LineStyle', ':');
    grid on

    daspect([1, 1, 1]);
    xlabel('[m]');
    ylabel('[m]');
    ax1 = gca();
    ax1.XTick = ax1.XLim(1):1.0:ax1.XLim(2);
    ax1.YTick = ax1.YLim(1):1.0:ax1.YLim(2);
end