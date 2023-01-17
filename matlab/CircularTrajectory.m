classdef CircularTrajectory

    properties
        center
        radius
        desired_driving_velocity
        desired_steering_velocity
        phi
        duration
    end
    
    methods
        function obj = CircularTrajectory(center, radius, desired_driving_velocity, phi, duration)
            obj.center = center;
            obj.radius = radius;
            obj.desired_driving_velocity = desired_driving_velocity;
            obj.desired_steering_velocity = desired_driving_velocity / radius;
            obj.phi = phi;
            obj.duration = duration;
        end
        
        function [pose, pose_derivative, pose_second_derivative] = eval(obj, time)
            xd = obj.center(1) + obj.radius * cos(obj.phi + obj.desired_steering_velocity * time);
            yd = obj.center(2) + obj.radius * sin(obj.phi + obj.desired_steering_velocity * time);

            xd_dot = -obj.radius * sin(obj.phi + obj.desired_steering_velocity * time) * obj.desired_steering_velocity;
            yd_dot =  obj.radius * cos(obj.phi + obj.desired_steering_velocity * time) * obj.desired_steering_velocity;
            
            xd_ddot = -obj.radius * cos(obj.phi + obj.desired_steering_velocity * time) * (obj.desired_steering_velocity ^ 2.0);
            yd_ddot = -obj.radius * sin(obj.phi + obj.desired_steering_velocity * time) * (obj.desired_steering_velocity ^ 2.0);

            % Compute using flat outputs:
            thetad = atan2(yd_dot, xd_dot);
            thetad_dot = (yd_ddot * xd_dot - xd_ddot * yd_dot) / (xd_dot ^ 2.0 + yd_dot ^ 2.0);
            thetad_ddot = 0.0;

            pose = [xd; yd; thetad];
            pose_derivative = [xd_dot; yd_dot; thetad_dot];
            pose_second_derivative = [xd_ddot; yd_ddot; thetad_ddot];
        end

        function desired_driving_velocity = getDesiredDrivingVelocity(obj, ~)
            desired_driving_velocity = obj.desired_driving_velocity;
        end

        function desired_steering_velocity = getDesiredSteeringVelocity(obj, ~)
            desired_steering_velocity = obj.desired_steering_velocity;
        end
    end
end

