classdef ApproximateLinearizationController
    properties
        zeta
        a
    end
    
    methods
        function obj = ApproximateLinearizationController(zeta, a)
            obj.zeta = zeta;
            obj.a = a;
        end
        
        function commands = compute_commands(obj, time, unicycle_configuration, desired_trajectory)
            x = unicycle_configuration(1);
            y = unicycle_configuration(2);
            theta = unicycle_configuration(3);
            
            [desired_pose, ~, ~] = desired_trajectory.eval(time);
            
            xd = desired_pose(1);
            yd = desired_pose(2);
            thetad = desired_pose(3);
            
            % NOTE: controller is stable iff vd and omegad are const.
            desired_driving_velocity = desired_trajectory.getDesiredDrivingVelocity(time);
            desired_steering_velocity = desired_trajectory.getDesiredSteeringVelocity(time);
            
            deltax = xd - x;
            deltay = yd - y;
            deltatheta = wrapToPi(thetad - theta);
            
            e1 =  cos(theta) * deltax + sin(theta) * deltay;
            e2 = -sin(theta) * deltax + cos(theta) * deltay;
            e3 = deltatheta;
            
            k1 = 2.0 * obj.zeta * obj.a;
            k2 = (obj.a ^ 2.0 - desired_steering_velocity ^ 2.0) / desired_driving_velocity;
            k3 = k1;
            
            u1 = -k1 * e1;
            u2 = -k2 * e2 - k3 * e3;
            
            driving_velocity = desired_driving_velocity * cos(e3) - u1;
            steering_velocity = desired_steering_velocity - u2;

            commands = [driving_velocity; steering_velocity];
        end
    end
end

