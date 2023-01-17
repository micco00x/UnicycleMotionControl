classdef StaticFeedbackLinearizationController
    properties
        b, k1, k2
    end
    
    methods
        function obj = StaticFeedbackLinearizationController(b, k1, k2)
            obj.b = b;
            obj.k1 = k1;
            obj.k2 = k2;
        end
        
        function commands = compute_commands(obj, time, unicycle_configuration, desired_trajectory)
            [desired_pose, desired_pose_derivative, ~] = desired_trajectory.eval(time);
            
            xd = desired_pose(1);
            yd = desired_pose(2);
            xd_dot = desired_pose_derivative(1);
            yd_dot = desired_pose_derivative(2);
            
            r1d = xd;
            r2d = yd;
            r1d_dot = xd_dot;
            r2d_dot = yd_dot;
            
            x = unicycle_configuration(1);
            y = unicycle_configuration(2);
            theta = unicycle_configuration(3);
            
            r1 = x + obj.b * cos(theta);
            r2 = y + obj.b * sin(theta);
            
            u1 = r1d_dot + obj.k1 * (r1d - r1);
            u2 = r2d_dot + obj.k2 * (r2d - r2);
            
            driving_velocity = cos(theta) * u1 + sin(theta) * u2;
            steering_velocity = -sin(theta) * u1 / obj.b + cos(theta) * u2 / obj.b;
            
            commands = [driving_velocity; steering_velocity];
        end
    end
end

