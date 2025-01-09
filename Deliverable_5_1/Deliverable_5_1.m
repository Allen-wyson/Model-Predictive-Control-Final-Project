%%%%%%%%%%%%%%%%%%%
% to produce figures of minimal invariant set / state constrained set /
% etc, run tube_mpc_sets
% or uncomment the bottom line!
%%%%%%%%%%%%%%%%%%%
clc
close all
clear all

addpath('..')

Ts = 0.1;
car = Car(Ts);
[xs, us] = car.steady_state(100/3.6);
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);

H_lon = 4;
mpc_lon = MpcControl_lon(sys_lon, Ts, H_lon);
mpc_lat = MpcControl_lat(sys_lat, Ts, H_lon);
% estimator = LonEstimator(sys_lon, Ts);
mpc = car.merge_lin_controllers(mpc_lon, mpc_lat);


% choose whether to simulate constant lead or random lead
lead_const = false;

if lead_const
    params={};
    params.Tf = 25;
    params.myCar.model = car;
    params.myCar.x0 = [0 0 0 100/3.6]';
    params.myCar.ref = [0 120/3.6]';
    params.myCar.u = @mpc.get_u;
    params.otherCar.model = car;
    params.otherCar.x0 = [15 0 0 100/3.6]';
    params.otherCar.u = car.u_const(100/3.6);
    result = simulate(params);
    visualization(car, result);
else
    params={};
    params.Tf = 25;
    params.myCar.model = car;
    params.myCar.x0 = [0 0 0 115/3.6];
    params.myCar.ref = [0 120/3.6]';
    params.myCar.u = @mpc.get_u;
    params.otherCar.model = car;
    params.otherCar.x0 = [8 0 0 120/3.6]';
    params.otherCar.u = car.u_fwd_ref();
    params.otherCar.ref = car.ref_robust();
    result = simulate(params);
    visualization(car, result);
end

% to obtain invariant set / terminal set figures
%uncomment below
%tube_mpc_sets