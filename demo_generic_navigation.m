clear; clc; close all;

addpath(genpath('../src'));

%% 1) Generic A* planning
grid = zeros(40,50);
grid(8:32,25) = 1;
grid(20,25) = 0;  % opening

startNode = [4 4];
goalNode  = [36 46];

path = astar_grid(grid,startNode,goalNode);

figure;
imagesc(grid);
axis equal tight;
set(gca,'YDir','normal');
hold on;
if ~isempty(path)
    plot(path(:,2),path(:,1),'LineWidth',1.5);
end
plot(startNode(2),startNode(1),'o');
plot(goalNode(2),goalNode(1),'x');
title('Generic A* Path');
xlabel('Column'); ylabel('Row');

%% 2) LOS heading to the next point
if size(path,1) >= 2
    pos = [path(1,2), path(1,1)];
    wp  = [path(2,2), path(2,1)];
    [psiRef,reached] = los_waypoint_guidance(pos,wp,0.5);
    fprintf('LOS heading reference: %.3f rad, reached=%d\n',psiRef,reached);
end

%% 3) Simple LQR example: planar double integrator
A = [0 1; 0 0];
B = [0; 1];
Q = diag([10 1]);
R = 1;

K = lqr_design(A,B,Q,R);
disp('LQR gain:');
disp(K);

%% 4) Luenberger observer
C = [1 0];
observerPoles = [-4 -5];
L = luenberger_design(A,C,observerPoles);
disp('Observer gain:');
disp(L);

%% 5) Discrete Kalman step
Ts = 0.05;
Ad = [1 Ts; 0 1];
Bd = [0.5*Ts^2; Ts];
Cd = [1 0];

xhat = [0;0];
P = eye(2);
u = 0.2;
y = 0.1;
Qk = 1e-3*eye(2);
Rk = 1e-2;

[xhat,P] = kalman_discrete_step(xhat,P,u,y,Ad,Bd,Cd,Qk,Rk);
disp('Updated Kalman state estimate:');
disp(xhat);

%% 6) Educational MRAC example
t = 0:0.01:10;
ref = ones(size(t));

params.a = -0.8;
params.b = 1.0;
params.am = -1.5;
params.bm = 1.5;
params.gamma = 2.0;
params.sigma = 0.05;

mrac = mrac_first_order(ref,t,params);

figure;
plot(t,mrac.y,'LineWidth',1.2); hold on;
plot(t,mrac.ym,'--','LineWidth',1.2);
grid on;
xlabel('Time [s]');
ylabel('Output');
legend('Plant','Reference model');
title('Generic First-Order MRAC Example');
