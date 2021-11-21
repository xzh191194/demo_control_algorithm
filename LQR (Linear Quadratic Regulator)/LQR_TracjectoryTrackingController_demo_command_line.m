%% Design Tracjectory Tracking Controller Using LQR at Command Line
% Plant: x''=3x-2u+d(t), d(t)=cos(2t^0.5)+sin(0.5t^2), d(t) is a unknown disturbance.
% Desired Signal: xr=sin(3t)
% Control Objective: x(t)-xr=0
% Initial Condition: x(0)=0

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
xr = zeros(2,N+1); % reference
x = zeros(2,N+1); % system states
e = zeros(2,N+1); % states error
u = zeros(1,N+1); % controller output
y = zeros(2,N+1); % system output
t = zeros(1,N+1); % clock
d = zeros(1,N+1); % disturbance

x(:,1) = [0 0]'; % define initial conditions
%xr(1,1) = 1;

%% design LQR Regulator
% weight matrix
Q = [1 0
     0 100];
R = 0.01;
K = lqr(A,B,Q,R);
disp('--------------------------');
fprintf('k1 = %.2f   ',K(1));
fprintf('k2 = %.2f \n',K(2));
disp('--------------------------');

%% iterate through each time step
disp('Start simulation...');

for k = 1:N
    
    t(1,k+1) = k*Ts;
    
    %xr(1,k+1) = 1;
    xr(1,k+1) = sin(3*t(1,k+1));
    xr(2,k+1) = (xr(1,k+1)-xr(1,k))/Ts; % desired signal derivative
    
    e(:,k) = x(:,k)-xr(:,k); % error state
    u(1,k) = -K*e(:,k)+3/2*xr(1,k); % control law
    
    %d(1,k) = 0;
    d(1,k) = cos(2*t(1,k)^0.5)+sin(0.5*t(1,k)^2); % disturbance
    
    E = [0;d(1,k)*Ts];
    
    x(:,k+1) = plant_d.A * x(:,k) + plant_d.B * u(1,k) + E;
    y(:,k) = plant_d.C * x(:,k);
    
end
    y(:,k+1) = plant_d.C * x(:,k+1);

%% plot figures
disp('Drawing...');

figure('Color','white')
plot(t,xr(1,:),'m--','linewidth',1.5); hold on;
plot(t,y(1,:),'k','linewidth',1.5); hold off;
grid on;
title('LQR Tracjectory Tracking');
xlabel('t/s'); ylabel('x')
legend('xr','x');

disp('End of the simulation.');

% Reference
% [1] https://zhuanlan.zhihu.com/p/58134063