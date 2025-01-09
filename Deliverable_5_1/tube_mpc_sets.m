clear all; clc; close all;
Ts = 0.1;
car = Car(Ts);
[xs, us] = car.steady_state(100/3.6);
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);
[f_xs_us, A, B, C, D] = Car.c2d_with_offset(sys_lon, Ts);
Q=diag([0.5, 1]); R=1;
[K,P] = dlqr(A,-B,Q,R);
K = -K; % Note that matlab defines K as -K
ABK = A+(-B)*K; % Closed-loop dynamics
Bd = B;
uT_s = us(2);
U_tilde = polytope([1;-1],[0.5+uT_s; 0.5-uT_s]);
figure(1); clf;
W = Bd*U_tilde;
O=W;
h1=plot(W, 'b');
hold on;
% H = [1 0; -1 0; 0 1; 0 -1];
% h = [Bd(1)*(uT_s + 0.5), Bd(1)*(-uT_s + 0.5), Bd(2)*(uT_s+0.5), Bd(2)*(-uT_s+0.5)]';
% W=polytope(H,h);
% h1=plot(W, 'b');
% hold on;
% Compute the maximal invariant set
while norm(ABK^i) > 1e-1
    W = ABK*W;
    O= plus(O, W);
	h2=plot(O, 'y'); 
    fprintf('Iteration %i... not yet equal\n', i) 
    
	i = i + 1;
end 
fprintf('Maximal control invariant set computed after %iiterations\n\n', i); 
h3=plot(O,'g'); legend([h3],{'Epsillon'});
Eps=O;
hold off

x_safe = 8;
% should I add v constraint? btwn -50~50 for example
% if there's no constraint on v, plot becomes strange
Hx = [-1 0; 0 1; 0 -1]; hx = [x_safe-6; 50; 50]; 
%Hx = [-1 0]; hx = x_safe - 6;

Hu = [1 ; -1]; hu = [1;1];
% do I include u=Kx constraint or not?
% X = polytope([Hx; Hu*K], [hx;hu]);
X = polytope(Hx, hx);

U = polytope(Hu, hu);
XminusEps = minus(X, Eps);
UminusKEps = minus(U, K*Eps);

i = 1;
O = XminusEps;
[F,f]=double(O);
O=polytope([F;Hu*K], [f;hu]); % add u=Kx constraint here?
while 1
    Oprev = O;
    [F, f] = double(O);	
    % Compute the pre-set
    O = polytope([F; F * ABK],[f; f]);
    if O == Oprev, break; end

    %h2 = plot(O, 'y');
    fprintf('Iteration %i... not yet equal\n', i)

    i = i + 1;

end
figure(); hold on;
h1=plot(X, 'b');
h2=plot(XminusEps, 'g');
h3=plot(O, 'y');
h4=plot(0,0,'r*');
legend('X', 'XminusEps', 'Xf', 'origin')
hold off

figure(); hold on;
h4=plot(U, 'r');
h5=plot(UminusKEps, 'b');
legend([h4;h5], {'U', 'UminusKEps'})
hold off

Xf = O;
[Eps_H, Eps_h] = double(Eps);
[X_H, X_h] = double(XminusEps);
[U_H, U_h] = double(UminusKEps);
[Xf_H, Xf_h] = double(Xf);
Qf = Q;
save('tube_mpc_data.mat', 'Eps_H', 'Eps_h', 'X_H', 'X_h', 'U_H', 'U_h','Xf_H', 'Xf_h','K', 'Qf', 'x_safe');