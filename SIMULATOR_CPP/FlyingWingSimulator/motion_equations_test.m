close all
clear all
clc

% opengl software
mex mexMotionTest.cpp plane_dynamics.cpp plane_mechanics.cpp -R2018a

a = arduino('COM6', 'ProMini328_5v'); %Define the arduino for communication from MATLAB to the arduino

 % iniatilize timing
t = 10.0; % max time to run
dt = 0.0001; %time step
time = 0:dt:t; %total time array
N = length(time);
rads = 45*pi/180;

u = 15.0; v = 0; w = 0; %Velocity in the x, y, and z directions
p = 0; q = 0; r = 0; %Roll, pitch, and yaw rate of the plane
pn = 0; %(m) North position in an inertial reference frame
pe = 0; %(m) East position in an inertial reference frame
pd = -10.0; %(m) Down position in an inertial reference frame
e1 = 0; e2 = 0; e3 = 0; e4 = 0; %quaternion angles
xold=[pn,pe,pd,u,v,w,e1,e2,e3,e4,p,q,r]; %State variable
phi = 0; theta = 0; psi = 0;
eulold = [phi, theta, psi];

figure
DrawAirplane(pn, pe, pd, phi, theta, psi);

% Enter simulation loop
for ii = 1:N
    h = sqrt(rads^2*2.0);
    x2Position = (readVoltage(a, 'A0')-2.5)*h/2.5;
    y2Position = (readVoltage(a, 'A1')-2.5)*h/2.5;
    xp = x2Position*cos(rads) + y2Position*sin(rads);
    yp = -x2Position*sin(rads) + y2Position*cos(rads);
    if xp > rads
        xp = rads; 
    elseif xp < -rads
        xp = -rads;  
    end
    if yp > rads
        yp = rads;  
    elseif yp < -rads
        yp = -rads;  
    end
    eLeft = xp;
    eRight = yp;
    throttle = (readVoltage(a, 'A2'))/5;%(0-1) Throttle  .6
        
    [xnew, eulnew] = mexMotionTest(dt,eLeft,eRight,throttle,xold,eulold);

    pn = xnew(1); pe = xnew(2); pd = xnew(3); %update the position
    u = xnew(4); v = xnew(5); w = xnew(6); %Update directional velocities
    e0 = xnew(7); e1 = xnew(8); e2 = xnew(9); e3 = xnew(10); %update quaternion angles
    p = xnew(11); q = xnew(12); r = xnew(13); % update roll, pitch, and yaw angle rates
    phi = eulnew(1); theta = eulnew(2); psi = eulnew(3);
    
    if mod(ii,2)==0
        clf('reset')
        DrawAirplane(pn, pe, pd, phi, theta, psi);
        pause(dt)
    end
    
    xold = xnew;
    eulold = eulnew;
end