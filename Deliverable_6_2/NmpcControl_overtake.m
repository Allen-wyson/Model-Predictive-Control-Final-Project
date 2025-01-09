classdef NmpcControl_overtake < handle

    properties
        % The NMPC problem
        opti

        % Problem parameters
        x0, ref, x0other, u_prev

        % Most recent problem solution
        sol

        % The input that you want to apply to the system
        u0

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Add any variables you would like to read to debug here
        % and then store them in the NmpcControl function below.
        % e.g., you could place X here and then add obj.X = X
        % in the NmpcControl function below.
        % 
        % After solving the problem, you can then read these variables 
        % to debug via
        %   nmpc.sol.value(nmpc.X)
        % 
        X, U
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    end

    methods
        function obj = NmpcControl_overtake(car, H)

            import casadi.*

            N_segs = ceil(H/car.Ts); % Horizon steps
            N = N_segs + 1;          % Last index in 1-based Matlab indexing

            nx = 4;
            nu = 2;

            % Define the NMPC optimization problem
            opti = casadi.Opti();
            
            % Parameters (symbolic)
            obj.x0 = opti.parameter(nx, 1);       % initial state
            obj.ref = opti.parameter(2, 1);       % target y, velocity
            obj.x0other = opti.parameter(nx, 1);  % initial state of other car
            obj.u_prev = opti.parameter(nu, 1);

            % SET THIS VALUE TO BE YOUR CONTROL INPUT
            obj.u0 = opti.variable(nu, 1);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            % ----- dynamics function -----
            f = @(x, u) car.f(x, u);

            % Define your problem using the opti object created above

            % state and input trajectory
            X = opti.variable(nx, N);
            U = opti.variable(nu, N);
            obj.X = X;
            obj.U = U;
            % xs = 
            % us = 

            % ----- cost function -----
            cost = 0;
            Q = diag([0.1, 1]);  % weights for y, V tracking
            R = diag([10, 1]);   % weights for inputs delta, uT

            for k = 1: N - 1
                tracking_error = [X(2, k) - obj.ref(1); X(4, k) - obj.ref(2)];
                cost = cost + tracking_error' * Q * tracking_error + U(:, k)' * R * U(:, k);
            end
            % cost = cost + (X(:, N) - xs)' * Q * (X(:, N) - obj.xs);

            % Terminal cost
            terminal_error = [X(2, N) - obj.ref(1); X(4, N) - obj.ref(2)];
            cost = cost + terminal_error' * Q * terminal_error;

            opti.minimize(cost);


            % ----- dynamics constraints -----
            for k = 1 : N-1
                opti.subject_to(X(:, k+1) == RK4(X(:, k), U(:, k), car.Ts, f));  % nonlinear dynamic function
            end

            % initial state
            opti.subject_to(X(:, 1) == obj.x0);

            % ----- state constraints -----
            opti.subject_to(-0.5 <= X(2, :) <= 3.5);    % lateral position
            opti.subject_to(-deg2rad(5) <= X(3, :) <= deg2rad(5));  % yaw

            % ----- input constraints -----
            opti.subject_to(-deg2rad(30) <= U(1, :) <= deg2rad(30));    % steering constraint
            opti.subject_to(-0.9999999 <= U(2, :) <= 0.9999999);    % throttle constraint
            
            % ----- comfort constraints -----
            max_throttle_rate = 0.4; % Maximum allowable change in throttle per step

            opti.subject_to(U(2, 1) - obj.u_prev(2) <= max_throttle_rate);
            opti.subject_to(obj.u_prev(2) - U(2, 1) <= max_throttle_rate);

            for k = 1:N-1
                opti.subject_to(U(2, k+1) - U(2, k) <= max_throttle_rate); % Positive rate limit
                opti.subject_to(U(2, k) - U(2, k+1) <= max_throttle_rate); % Negative rate limit
            end


            % ----- collision avoidance -----
            for k = 1 : N
                p = [X(1, k); X(2, k)];

                % Predict the position of the other car for time step k
                p_L = [obj.x0other(1) + obj.x0other(4) * cos(obj.x0other(3)) * k * car.Ts; % Longitudinal
                         obj.x0other(2) + obj.x0other(4) * sin(obj.x0other(3)) * k * car.Ts]; % Lateral
                a = 8; % Semi-major axis of the ellipsoid
                b = 3; % Semi-minor axis of the ellipsoid

                H = diag([1/a^2, 1/b^2]); % Ellipse matrix
                % ((x - xc) / a^2) + ((y - yc) / b^2) = 1

                opti.subject_to((p - p_L)' * H * (p - p_L) >= 1);

                % % Add penalty to cost for soft collision constraint
                % dist_to_ellipse = 1 - (p - p_L)' * H * (p - p_L);
                % opti.subject_to(dist_to_ellipse >= 0.1); % Add a safety margin
                % 
                % cost = cost + lambda * max(0, dist_to_ellipse)^2;
            end


            % set the 1st control input
            opti.subject_to( obj.u0 == U(:, 1) );

            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Store the defined problem to solve in get_u
            obj.opti = opti;

            % Setup solver
            options = struct;
            options.ipopt.print_level = 0;
            options.print_time = 0;
            options.expand = true;
            obj.opti.solver('ipopt', options);
        end

        function u = get_u(obj, x0, ref, x0other, u_prev)
            if nargin < 5
                u_prev = zeros(2, 1);
            end

            if nargin < 4
                x0other = zeros(4, 1);
            end

            % Compute solution from x0
            obj.solve(x0(1:4), ref, x0other(1:4), u_prev);

            u = obj.sol.value(obj.u0);
        end

        function solve(obj, x0, ref, x0other, u_prev)

            % Pass parameter values
            obj.opti.set_value(obj.x0, x0);
            obj.opti.set_value(obj.ref, ref);
            obj.opti.set_value(obj.x0other, x0other);
            obj.opti.set_value(obj.u_prev, u_prev)

            obj.sol = obj.opti.solve();   % actual solve
            
            % Set warm start for next solve
            obj.opti.set_initial(obj.sol.value_variables());
            obj.opti.set_initial(obj.opti.lam_g, obj.sol.value(obj.opti.lam_g));
        end
    end
end
