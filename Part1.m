addpath('C:\Users\allen\Desktop\Coding\casadi-3.6.7-windows64-matlab2018b')
import casadi.*

%% Todo 1.1
Ts = 1 / 10;
car = Car(Ts);

delta = 1;
u_T = 0.7;
x = 2;
y = 2;
theta = pi;
V = 5;

u = [delta u_T]';
x = [x y theta V]';
x_dot = car.f(x, u);

%% Todo 1.2
car = Car(Ts);
Tf = 2.0;                           % Simulation end time

steering_angle = -10;
throttle = 0.7;

x0 = [0 0 deg2rad(-2) 20/3.6]';     % (x, y, theta, V) Initial state
u = [deg2rad(steering_angle) throttle]';              % (delta, u_T) Constant input

params = {};                        % Setup simulation parameter struct
params.Tf = Tf;
params.myCar.model = car;
params.myCar.x0 = x0;
params.myCar.u = u;
result = simulate(params);          % Simulate nonlinear model
visualization(car, result);
