classdef MpcControl_lon < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x0           - initial state (estimate)
            %   V_ref, u_ref - reference state/input
            %   d_est        - disturbance estimate
            %   x0other      - initial state of other car
            % OUTPUTS
            %   u0           - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            load('tube_mpc_data.mat'); 
            % 'Eps_H', 'Eps_h', 'X_H', 'X_h', 'U_H', 'U_h','Xf_H', 'Xf_h','K', 'Qf', 'x_safe'
            N_segs = ceil(mpc.H/mpc.Ts); % Horizon steps
            N = N_segs + 1;              % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets
            V_ref = sdpvar(1);
            u_ref = sdpvar(1);

            % Disturbance estimate (Ignore this before Todo 4.1)
            d_est = sdpvar(1);

            % Initial states
            x0 = sdpvar(nx, 1);
            x0other = sdpvar(nx, 1); % (Ignore this before Todo 5.1)
            % Input to apply to the system
            u0 = sdpvar(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D
            %       are the DISCRETE-TIME MODEL of your system.
            %       You can find the linearization steady-state
            %       in mpc.xs and mpc.us.
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            obj = 0;
            con = [];
            Z = sdpvar(nx,N);
            v = sdpvar(nu, N-1);
            Q = 1; % do not penalize x, only consider V
            R = 0.1;
            A = mpc.A;
            B = mpc.B;
            xs = mpc.xs;
            us = mpc.us;
            f_xs_us = mpc.f_xs_us;
            
            % let z = delta + e, delta = x0other - x - xsafe
            for k = 1:N-1
                con = con + (Z(:, k+1) == A*Z(:, k) - B*v(:,k)); % delta dynamics, mind B sign!
                con = [con, U_H*v(:, k) <= U_h];
                con = [con, X_H*Z(:,k) <= X_h];
                % cost for tracking ref velocity, TODO
                obj = obj + Q*(x0other(2)-Z(2, k)-V_ref)^2 + ...
                                (v(:, k)-u_ref)' * R * (v(:, k)-u_ref ); % V to track V_ref, v to track u_ref
            end
            con = con + (Xf_H*Z(:,N) <= Xf_h); %terminal constraint for Z(N)
            obj = obj + Q*(x0other(2) - Z(2,N)-V_ref)^2; % terminal cost
            con = con + ( Eps_H*(x0other - x0 - [x_safe;0]-Z(:,1)) <= Eps_h); % first tube center constraint
            con = con + ( u0 == (v(:,1)+K*(x0other - x0 - [x_safe;0] - Z(:,1)) )); % control law

            debugVars = {Z,v};
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','quadprog'), ...
                {x0, V_ref, u_ref, d_est, x0other}, {u0, debugVars{:}});
        end
        
        % Computes the steady state target which is passed to the
        % controller
        function [Vs_ref, us_ref] = compute_steady_state_target(mpc, ref, d_est)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            %   d_est  - disturbance estimate (Ignore before Todo 4.1)
            % OUTPUTS
            %   Vs_ref, us_ref - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Steady-state subsystem
            A = mpc.A(2, 2);
            B = mpc.B(2, 1);

            % Subsystem linearization steady-state
            xs = mpc.xs(2);
            us = mpc.us;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            u = sdpvar(1);
            Vs_ref = ref; 
            umin = -1;
            umax = 1;
            constraints = [ umin <= u <= umax, ...
                            Vs_ref == mpc.f_xs_us(2) + A*(Vs_ref - xs) + B*(u - us) ];
            
            objective   = u^2;
            diagnostics = solvesdp(constraints,objective,sdpsettings('verbose',0));
            if diagnostics.problem == 0
               % Good! 
            elseif diagnostics.problem == 1
                throw(MException('','Infeasible'));
            else
                throw(MException('','Something else happened'));
            end
            us_ref = double(u);
            Vs_ref = double(Vs_ref);
            %fprintf('Vs_ref %.2d\n', Vs_ref)
            %fprintf('us_ref %.2d\n', us_ref)
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
