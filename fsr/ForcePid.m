classdef ForcePid < handle
    properties
        Kp = 0.8
        Ki = 0
        Kd = 0

        integral = 0
        prevError = 0

        outputLimit = 0.02
        integralLimit = inf
    end

    methods

        function u = update(obj, error, dt)

            % disp("error = ");
            % disp(error);

            % Accumulate integral error
            obj.integral = obj.integral + error * dt;

            % Anti-windup clamp
            obj.integral = max(min(obj.integral, obj.integralLimit), ...
                               -obj.integralLimit);

            % Derivative term
            derivative = (error - obj.prevError) / dt;

            % PID output
            u = obj.Kp * error + ...
                obj.Ki * obj.integral + ...
                obj.Kd * derivative;

            % Output clamp
            % disp("u = ");
            % disp(u);
            u = max(min(u, obj.outputLimit), -obj.outputLimit);

            obj.prevError = error;
        end

        function reset(obj)
            obj.integral = 0;
            obj.prevError = 0;
        end
    end
end