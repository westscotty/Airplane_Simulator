%The WindTunnelTester tests the functions LiftAndDrag and LateralDynamics

close all
clear all
clc

%% test LiftAndDrag to see the effects of the elevator angle (delta_e)
mex mexWindTunnel.cpp -R2018a
test_type = 0;
n = 100;
q = 0;
alpha = 0*pi/180*[...
    -20*ones(1,n);...
    -10*ones(1,n);...
    0*ones(1,n);...
    10*ones(1,n);...
    20*ones(1,n)]'; %(rad) angle of attack
delta_e = pi/180*[...
    -20*ones(1,n);...
    -10*ones(1,n);...
    0*ones(1,n);...
    10*ones(1,n);...
    20*ones(1,n)]';
Va = [...
    linspace(0,30,n);...
    linspace(0,30,n);...
    linspace(0,30,n);...
    linspace(0,30,n);...
    linspace(0,30,n)]';

[~, ~, Ty] = mexWindTunnel(test_type, q, alpha, delta_e, Va);
figure, plot(Va, Ty)
xlabel('Airspeed (V_a) (m/s)')
ylabel('Pitch Torque (N m)')
title('Effect of Elevator Angle')
legend('\delta_e=-20','\delta_e=-10','\delta_e=0',...
    '\delta_e=10','\delta_e=20')

%% test the effect of the angle of attack (alpha)

test_type = 0;
delta_e = 0*pi/180*[...
    -20*ones(1,n);...
    -10*ones(1,n);...
    0*ones(1,n);...
    10*ones(1,n);...
    20*ones(1,n)]'; %(rad) elevator angle
alpha = pi/180*[...
    -20*ones(1,n);...
    -10*ones(1,n);...
    0*ones(1,n);...
    10*ones(1,n);...
    20*ones(1,n)]';
Va = [...
    linspace(0,30,n);...
    linspace(0,30,n);...
    linspace(0,30,n);...
    linspace(0,30,n);...
    linspace(0,30,n)]';
[fx, fz, Ty] = mexWindTunnel(test_type, q, alpha, delta_e, Va);

figure, plot(Va, fx)
xlabel('Airspeed (V_a) (m/s)')
ylabel('Drag (N)')
title('Effect of Angle of attack')
legend('\alpha=-20','\alpha=-10','\alpha=0',...
    '\alpha=10','\alpha=20')

figure, plot(Va, fz)
xlabel('Airspeed (V_a) (m/s)')
ylabel('Lift (N)')
title('Effect of Angle of attack')
legend('\alpha=-20','\alpha=-10','\alpha=0',...
    '\alpha=10','\alpha=20')

figure, plot(Va, Ty)
xlabel('Airspeed (V_a) (m/s)')
ylabel('Pitch Torque (N m)')
title('Effect of Angle of attack')
legend('\alpha=-20','\alpha=-10','\alpha=0',...
    '\alpha=10','\alpha=20')


%% test the effects of the aileron (delta_a)

test_type = 1;
beta = 0; %(rad) side slip angle
p=0;
r=0;
delta_r = 0*pi/180*[...
    -20*ones(1,n);...
    -10*ones(1,n);...
    0*ones(1,n);...
    10*ones(1,n);...
    20*ones(1,n)]';
delta_a = pi/180*[...
    -20*ones(1,n);...
    -10*ones(1,n);...
    0*ones(1,n);...
    10*ones(1,n);...
    20*ones(1,n)]';
Va = [...
    linspace(0,30,n);...
    linspace(0,30,n);...
    linspace(0,30,n);...
    linspace(0,30,n);...
    linspace(0,30,n)]';

[~, Tx, ~] = mexWindTunnel(test_type, beta, p, r, delta_r, delta_a, Va);

figure, plot(Va, Tx)
xlabel('Airspeed (V_a) (m/s)')
ylabel('Roll torque (N m)')
title('Effect of the Aileron Angle')
legend('\delta_a=-20','\delta_a=-10','\delta_a=0',...
    '\delta_a=10','\delta_a=20')