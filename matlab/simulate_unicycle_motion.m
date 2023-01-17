function unicycle_configuration= simulate_unicycle_motion(unicycle_configuration, control_input, sampling_interval)
    %unicycle_configuration = euler_integration(unicycle_configuration, control_input, sampling_interval);

    x = unicycle_configuration(1);
    y = unicycle_configuration(2);
    theta = unicycle_configuration(3);
    driving_velocity = control_input(1);
    steering_velocity = control_input(2);
    
    if abs(steering_velocity) < 1e-3
        % 2-nd order RK:
        unicycle_configuration(1) = x + driving_velocity * sampling_interval * cos(theta + steering_velocity * sampling_interval / 2.0);
        unicycle_configuration(2) = y + driving_velocity * sampling_interval * sin(theta + steering_velocity * sampling_interval / 2.0);
        unicycle_configuration(3) = theta + steering_velocity * sampling_interval;
    else
        % Exact integration of the unicycle:
        theta_next = theta + steering_velocity * sampling_interval;
        unicycle_configuration(1) = x + driving_velocity / steering_velocity * (sin(theta_next) - sin(theta));
        unicycle_configuration(2) = y - driving_velocity / steering_velocity * (cos(theta_next) - cos(theta));
        unicycle_configuration(3) = theta_next;
    end
end
