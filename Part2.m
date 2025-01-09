addpath('C:\Users\allen\Desktop\Coding\casadi-3.6.7-windows64-matlab2018b')
import casadi.*

%% Todo 2.1
Ts = 1 / 10;
car = Car(Ts);

Vs = 120 / 3.6;   % 120 km/h
[xs, us] = car.steady_state(Vs);    % Compute steady-state for which f_s(xs, us) = 0
sys = car.linearize(xs, us);    % Linearize the nonlinear model around xs, us

%% Todo 2.2
% Decompose into subsystems
[sys_lon, sys_lat] = car.decompose(sys);
[fd_xs_us, Ad, Bd, Cd, Dd] = Car.c2d_with_offset(sys, Ts);