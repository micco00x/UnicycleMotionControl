classdef EightShapedTrajectory
    properties
        starting_pose
        desired_steering_velocity
        duration
        R1
        R2
    end
    
    methods
        function obj = EightShapedTrajectory(starting_pose, desired_steering_velocity)
            obj.starting_pose = starting_pose;
            obj.desired_steering_velocity = desired_steering_velocity;
            obj.duration = 4.0 * pi / desired_steering_velocity;

            obj.R1 = 3.0;
            obj.R2 = 3.0;
        end
        
        function [pose, pose_derivative, pose_second_derivative] = eval(obj, time)
            xd = obj.starting_pose(1) + obj.R1 * sin(obj.desired_steering_velocity * time);
            yd = obj.starting_pose(2) + obj.R2 + obj.R2 * cos(pi + 0.5 * obj.desired_steering_velocity * time);

            xd_dot = obj.R1 * obj.desired_steering_velocity * cos(obj.desired_steering_velocity * time);
            yd_dot = -0.5 * obj.R2 * obj.desired_steering_velocity * sin(pi + 0.5 * obj.desired_steering_velocity * time);

            xd_ddot = -obj.R1 * obj.desired_steering_velocity ^ 2.0 * sin(obj.desired_steering_velocity * time);
            yd_ddot = -0.25 * obj.R2 * obj.desired_steering_velocity ^ 2.0 * cos(pi + 0.5 * obj.desired_steering_velocity * time);

            % Compute using flat outputs:
            thetad = atan2(yd_dot, xd_dot);
            thetad_dot = (yd_ddot * xd_dot - xd_ddot * yd_dot) / (xd_dot ^ 2.0 + yd_dot ^ 2.0);
            thetad_ddot = 0.0; % TODO.

            pose = [xd; yd; thetad];
            pose_derivative = [xd_dot; yd_dot; thetad_dot];
            pose_second_derivative = [xd_ddot; yd_ddot; thetad_ddot];
        end

        function desired_driving_velocity = getDesiredDrivingVelocity(obj, time)
            % NOTE: safe to do this because assuming the robot is only moving forward.
            [~, pose_derivative, ~] = obj.eval(time);
            xd_dot = pose_derivative(1);
            yd_dot = pose_derivative(2);
            desired_driving_velocity = sqrt(xd_dot ^ 2.0 + yd_dot ^ 2.0);
        end

        function desired_steering_velocity = getDesiredSteeringVelocity(obj, time)
            [~, pose_derivative, ~] = obj.eval(time);
            desired_steering_velocity = pose_derivative(3);
        end
    end
end

