%% Deliverable 2.1

%% Symbolic variables for state, input, and constants
% State and input variables
syms x  % x position
syms y  % y position
syms theta  % yaw angle
syms V  % velocity
syms delta  % steering angle
syms uT % normalized throttle

% Physical constants
syms lr % distance from the center of mass to the rear axles
syms lf % distance from the center of mass to the front axles
syms Pmax   % maximum motor power
syms rho    % air density
syms Cd % drag coefficient
syms Af % frontal area
syms Cr % rolling resistance
syms m  % mass
syms g  % gravity

%% Define the dynamics equations
% Slip angle
beta = atan((lr * tan(delta)) / (lr + lf));

% State equations
f1 = V * cos(theta + beta);  % dx/dt
f2 = V * sin(theta + beta);  % dy/dt
f3 = (V / lr) * sin(beta);   % dtheta/dt
f4 = (uT * Pmax / V - (1/2) * rho * Cd * Af * V^2 - Cr * m * g) / m;  % dV/dt

% Combine into state function f
f = [f1; f2; f3; f4];

% Steady-state conditions
xs = [0; 0; 0; sym('Vs')];  % Steady-state state [0, 0, 0, Vs]
us = [0; sym('uT_s')];      % Steady-state input [0, uT_s]

% Linearization: Compute Jacobians
A = jacobian(f, [x, y, theta, V]);  % Partial derivatives of f w.r.t. state variables
B = jacobian(f, [delta, uT]);       % Partial derivatives of f w.r.t. input variables

% Substitute steady-state values into A and B and f
A_steady = subs(A, [x, y, theta, V, delta, uT], [xs.', us.']);
B_steady = subs(B, [x, y, theta, V, delta, uT], [xs.', us.']);
f_steady = subs(f, [x, y, theta, V, delta, uT], [xs.', us.']);

% Simplify results for clarity
A_steady = simplify(A_steady);
B_steady = simplify(B_steady);
f_steady = simplify(f_steady);

% Display the results
disp('Linearized f at steady state:');
disp(f_steady);
disp('Linearized A matrix at steady state:');
disp(A_steady);
disp('Linearized B matrix at steady state:');
disp(B_steady);
