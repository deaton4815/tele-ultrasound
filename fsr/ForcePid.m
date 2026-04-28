classdef ForcePID < handle
    properties
        Kp = 0
        Ki = 0
        Kd = 0

        dt = 0.01
        integral = 0
        prevError = 0

        outputLimit = inf
        integralLimit = inf
    end

    methods
        function obj = ForcePID(Kp, Ki, Kd, dt)
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            obj.dt = dt;
        end

        function u = update(obj, error)
            % Accumulate integral error
            obj.integral = obj.integral + error * obj.dt;

            % Anti-windup clamp
            obj.integral = max(min(obj.integral, obj.integralLimit), ...
                               -obj.integralLimit);

            % Derivative term
            derivative = (error - obj.prevError) / obj.dt;

            % PID output
            u = obj.Kp * error + ...
                obj.Ki * obj.integral + ...
                obj.Kd * derivative;

            % Output clamp
            u = max(min(u, obj.outputLimit), -obj.outputLimit);

            obj.prevError = error;
        end

        function reset(obj)
            obj.integral = 0;
            obj.prevError = 0;
        end
    end
end