classdef MpcControl_lat < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x0           - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   u0           - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(mpc.H/mpc.Ts); % Horizon steps
            N = N_segs + 1;              % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            % Targets
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);

            % Initial states
            x0 = sdpvar(nx, 1);
            x0other = sdpvar(nx, 1); % (Ignore this, not used)

            % Input to apply to the system
            u0 = sdpvar(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D
            %       are the DISCRETE-TIME MODEL of your system
            %       You can find the linearization steady-state
            %       in mpc.xs and mpc.us.
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            xs = mpc.xs;
            us = mpc.us;
            Xf = terminal_invariant_set(mpc);
            Q = diag([10,1]); % y, theta
            R = 0.5; % steering angle

            delta_max = deg2rad(30);
            y_min = -0.5;
            y_max = 3.5;
            theta_max = deg2rad(5);
            obj = 0;
            con = [];
            for k = 1:N-1
                con = con + (X(:, k+1) == mpc.f_xs_us +mpc.A*(X(:, k) - xs) + mpc.B*(U(:,k)-us));
                con = con + (-delta_max <= U(:,k) <= delta_max);
                con = con + (y_min <= X(1, k) <= y_max);
                con = con + (-theta_max <= X(2,k) <= theta_max);
                obj = obj + (X(:,k) - x_ref)' * Q * (X(:,k) - x_ref) + ...
                                (U(:,k) - u_ref)' * R * (U(:,k) - u_ref);
            end
            obj = obj + (X(:, N) - x_ref)' * Q * (X(:, N) - x_ref);
            con = con + (X(:,1) == x0);
            con = con + (U(:,1) == u0);
            debugVars = {X, U};
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {x0, x_ref, u_ref, x0other}, {u0, debugVars{:}});
        end
        
        % Computes the steady state target which is passed to the
        % controller
        function [xs_ref, us_ref] = compute_steady_state_target(mpc, ref)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs_ref, us_ref - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Steady-state system
            A = mpc.A;
            B = mpc.B;

            % Linearization steady-state
            xs = mpc.xs;
            us = mpc.us;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            y_ref = ref; 
            xs_ref = [y_ref; 0]; % y_ref, zero theta
            us_ref = 0; % zero steering
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        function O = terminal_invariant_set(mpc)
            A = mpc.A;
            B = mpc.B;
            K = dlqr(A,B,eye(2),1);
            K = -K; % Note that matlab defines K as -K
            Ak = A+B*K; % Closed-loop dynamics
            ys = mpc.xs(1); theta_s = mpc.xs(2); us = mpc.us(1);
            H = [1 0; -1 0; 0 1; 0 -1];
            h = [3.5 - ys, 0.5 + ys, deg2rad(5) - theta_s, deg2rad(5) + theta_s]';
            Hu = [1; -1]; hu = [deg2rad(30) - us; deg2rad(30) + us];
            HH = [H;Hu*K]; hh = [h;hu]; % State and input constraints
            
            h4=plot(polytope(HH,hh), 'c');
            hold on;
            
            % Compute the maximal invariant set
            i = 1;
            O = polytope(HH,hh);
            while 1
	            Oprev = O;
	            [F,f] = double(O);	
	            % Compute the pre-set
	            O = polytope([F;F*Ak],[f;f]);
	            if O == Oprev, break; end
	            
	            h2=plot(O, 'y');
	            fprintf('Iteration %i... not yet equal\n', i)
           
	            i = i + 1;
            end
        end
    end
end
