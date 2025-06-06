clc;
close all;
clear;

addpath('..')

%%
Ts = 0.1;
car = Car(Ts);
[xs, us] = car.steady_state(120/3.6);
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);

H_lon = 5;
mpc_lon = MpcControl_lon(sys_lon, Ts, H_lon);
mpc_lat = MpcControl_lat(sys_lat, Ts, H_lon);
estimator = LonEstimator(sys_lon, Ts);
mpc = car.merge_lin_controllers(mpc_lon, mpc_lat);
%%
x0 = [0,0,0,80/3.6]';
ref1 = [0, 80/3.6]'; % y_ref, V_ref
ref2 = [3 50/3.6]'; % y_ref, V_ref

params={};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = x0;
params.myCar.est_fcn = @estimator.estimate;
params.myCar.est_dist0 = 0;
params.myCar.u = @mpc.get_u;
params.myCar.ref = car.ref_step(ref1, ref2, 2);
result = simulate(params);
visualization(car, result);