classdef DynamicFeedbackLinearizationController < handle
    properties
        kp1, kp2, kd1, kd2
        sampling_interval
        xi_prev
    end
    
    methods
        function obj = DynamicFeedbackLinearizationController(kp1, kp2, kd1, kd2, sampling_interval, xi_0)
            obj.kp1 = kp1;
            obj.kp2 = kp2;
            obj.kd1 = kd1;
            obj.kd2 = kd2;
            obj.sampling_interval = sampling_interval;
            obj.xi_prev = xi_0;
        end
        
        function commands = compute_commands(obj, time, unicycle_configuration, unicycle_velocity, desired_trajectory)
            x = unicycle_configuration(1);
            y = unicycle_configuration(2);
            theta = unicycle_configuration(3);
            
            x_dot = unicycle_velocity(1);
            y_dot = unicycle_velocity(2);
            
            [desired_pose, desired_pose_derivative, desired_pose_second_derivative] = desired_trajectory.eval(time);
            
            xd = desired_pose(1);
            yd = desired_pose(2);
            xd_dot = desired_pose_derivative(1);
            yd_dot = desired_pose_derivative(2);
            xd_ddot = desired_pose_second_derivative(1);
            yd_ddot = desired_pose_second_derivative(2);
            
            % PD controller:
            u1 = xd_ddot + obj.kp1 * (xd - x) + obj.kd1 * (xd_dot - x_dot);
            u2 = yd_ddot + obj.kp2 * (yd - y) + obj.kd2 * (yd_dot - y_dot);
            
            % Dynamic compensator:
            xi_dot = u1 * cos(theta) + u2 * sin(theta);
            xi = obj.xi_prev + xi_dot * obj.sampling_interval; % integrator
            driving_velocity = xi;
            obj.xi_prev = xi;
            steering_velocity = (u2 * cos(theta) - u1 * sin(theta)) / xi;
            
            commands = [driving_velocity; steering_velocity];
        end
    end
end

