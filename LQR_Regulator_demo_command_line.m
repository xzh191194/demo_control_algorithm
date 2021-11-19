%% Design Regulator Using LQR at Command Line
% Plant: x''=3x-2u+d(t), d(t)=sin(0.1t), d(t) is a unknown disturbance.
% Desired Signal: xr=0
% Control Objective: x(t)->0
% Initial Condition: x(0)=2

% Designed by Chiled_JiuAn.

%% simulation environment initialization
clc; close all; clear;

%% create state space plant model
A = [0 1;
     3 0];
B = [0;
     -2];
C = [1 0;
     0 0];
D = 0;

plant = ss(A,B,C,D);

Ts = 0.01; % sampling time
simTime = 10; % simulation time
N = simTime/Ts;

plant_d = c2d(plant,Ts); % discretization

%% initial states
xr = zeros(1,N+1); % reference
x = zeros(2,N+1); % system states
u = zeros(1,N+1); % controller output
y = zeros(2,N); % system output
t = zeros(1,N+1); % clock
d = zeros(1,N+1); % disturbance

x(:,1) = [2 0]'; % define initial conditions

%% design LQR Regulator
% weight matrix
Q = [100 0
     0   1];
R = 0.01;
K = lqr(A,B,Q,R);
disp('--------------------------');
fprintf('k1 = %.2f   ',K(1));
fprintf('k2 = %.2f \n',K(2));
disp('--------------------------');

%% iterate through each time step
disp('Start simulation...');

for i = 1:N
    t(1,i+1) = i*Ts; 
    u(1,i) = -K*x(:,i); % state feedback
    d(1,i+1) = sin(0.1*t(1,i+1));
    E = [0;d(1,i+1)*Ts];
    x(:,i+1) = plant_d.A * x(:,i) + plant_d.B * u(1,i) + E;
    y(:,i) = plant_d.C * x(:,i);
end

%% plot figures
disp('Drawing...');

figure('Color','white')
plot(t,x(1,:),'k','linewidth',1.5); grid on;
title('LQR Regulator');
xlabel('t/s'); ylabel('x')

disp('End of the simulation.');

% Reference
% [1] https://zhuanlan.zhihu.com/p/58134063





























