classdef SquaredTrajectory
    properties
        desired_driving_velocity
        duration
        x0, y0, theta0
        x1, y1, theta1
        x2, y2, theta2
        x3, y3, theta3
        t0, t1, t2, t3, t4
    end
    
    methods
        function obj = SquaredTrajectory(t0, square_frame, desired_driving_velocity, square_length)
            obj.desired_driving_velocity = desired_driving_velocity;

            obj.x0 = square_frame(1);
            obj.y0 = square_frame(2);

            obj.theta0 = square_frame(3);
            obj.theta1 = obj.theta0 + pi * 0.5;
            obj.theta2 = obj.theta0 + pi;
            obj.theta3 = obj.theta0 + pi * 1.5;

            obj.x1 = obj.x0 + square_length * cos(obj.theta0);
            obj.y1 = obj.y0 + square_length * sin(obj.theta0);
            obj.x2 = obj.x1 + square_length * cos(obj.theta1);
            obj.y2 = obj.y1 + square_length * sin(obj.theta1);
            obj.x3 = obj.x2 + square_length * cos(obj.theta2);
            obj.y3 = obj.y2 + square_length * sin(obj.theta2);

            obj.t0 = t0;
            obj.t1 = obj.t0 + square_length / obj.desired_driving_velocity;
            side_to_side_time = obj.t1 - obj.t0;
            obj.t2 = obj.t1 + side_to_side_time;
            obj.t3 = obj.t2 + side_to_side_time;
            obj.t4 = obj.t3 + side_to_side_time;

            obj.duration = obj.t4 - obj.t0;
        end
        
        function [pose, pose_derivative, pose_second_derivative] = eval(obj, time)
            if time < obj.t0
                xd = obj.x0;
                yd = obj.y0;
                xd_dot = 0.0;
                yd_dot = 0.0;
            elseif obj.t0 <= time && time <= obj.t1
                xd = obj.x0 + obj.desired_driving_velocity * cos(obj.theta0) * (time - obj.t0);
                yd = obj.y0 + obj.desired_driving_velocity * sin(obj.theta0) * (time - obj.t0);
                xd_dot = obj.desired_driving_velocity * cos(obj.theta0);
                yd_dot = obj.desired_driving_velocity * sin(obj.theta0);
            elseif obj.t1 <= time && time <= obj.t2
                xd = obj.x1 + obj.desired_driving_velocity * cos(obj.theta1) * (time - obj.t1);
                yd = obj.y1 + obj.desired_driving_velocity * sin(obj.theta1) * (time - obj.t1);
                xd_dot = obj.desired_driving_velocity * cos(obj.theta1);
                yd_dot = obj.desired_driving_velocity * sin(obj.theta1);
            elseif obj.t2 <= time && time <= obj.t3
                xd = obj.x2 + obj.desired_driving_velocity * cos(obj.theta2) * (time - obj.t2);
                yd = obj.y2 + obj.desired_driving_velocity * sin(obj.theta2) * (time - obj.t2);
                xd_dot = obj.desired_driving_velocity * cos(obj.theta2);
                yd_dot = obj.desired_driving_velocity * sin(obj.theta2);
            elseif obj.t3 <= time && time <= obj.t4
                xd = obj.x3 + obj.desired_driving_velocity * cos(obj.theta3) * (time - obj.t3);
                yd = obj.y3 + obj.desired_driving_velocity * sin(obj.theta3) * (time - obj.t3);
                xd_dot = obj.desired_driving_velocity * cos(obj.theta3);
                yd_dot = obj.desired_driving_velocity * sin(obj.theta3);
            else
                xd = obj.x0;
                yd = obj.y0;
                xd_dot = 0.0;
                yd_dot = 0.0;
            end

            thetad = atan2(yd_dot, xd_dot);
            thetad_dot = 0.0;

            pose = [xd, yd, thetad];
            pose_derivative = [xd_dot, yd_dot, thetad_dot];
            pose_second_derivative = [0.0, 0.0, 0.0];
        end
    end
end

