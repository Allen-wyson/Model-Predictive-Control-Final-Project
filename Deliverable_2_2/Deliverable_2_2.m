%% Deliverable 2.2
%  Explain from an intuitive physical / mechanical perspective, 
%  why this separation into independent subsystems is possible.


%% Explaination
clc
clear all
close all
addpath('..')

Ts=0.1;
car = Car(Ts);
Vs = 120/3.6;
[xs, us] = car.steady_state(Vs);
sys = car.linearize(xs, us);
% [fd_xs_us, Ad, Bd, Cd, Dd] = Car.c2d_with_offset(sys, Ts);
[sys_lon, sys_lat] = car.decompose(sys);
disp('A matrix')
disp(sys.A)
disp('B matrix')
disp(sys.B)
fprintf(['from structure of A and B \nwe can see that uT influences V and V influences x and V\n' ...
    'steering angle influences y and theta, and theta influences y\n' ...
    '(x,V, uT) and (y,theta,steering angle) do not influence each other\n' ...
    'when sysem is linearized around steady cruising motion\n' ...
    'Thus they can be split into lon and lat system\n\n'])
%% showing that decomposing and composing preserves info
X = xs; 
U = us+[0;0.1];

x_lon_idx = [1,4];
x_lat_idx = [2,3];
u_lon_idx = 2;
u_lat_idx = 1;

X_lon = X(x_lon_idx);
X_lat = X(x_lat_idx);
U_lon = U(u_lon_idx);
U_lat = U(u_lat_idx);
xs_lon = xs(x_lon_idx);
xs_lat = xs(x_lat_idx);
us_lon = us(u_lon_idx);
us_lat = us(u_lat_idx);
X_dot = car.f(xs, us) + sys.A * (X - xs) + sys.B * (U - us) ;
X_lon_dot = sys_lon.A * (X_lon - xs_lon) + sys_lon.B * (U_lon-us_lon) ;
X_lat_dot = sys_lat.A * (X_lat - xs_lat) + sys_lat.B * (U_lat - us_lat);

X_dot_compose = zeros(4,1);
X_dot_compose(x_lon_idx) = X_lon_dot;
X_dot_compose(x_lat_idx) = X_lat_dot;
X_dot_compose = X_dot_compose + car.f(xs,us);

disp('Original Xdot')
disp(X_dot)
disp('Xdot composed from sys_lon and sys_lat')
disp(X_dot_compose)
disp('they are identical')