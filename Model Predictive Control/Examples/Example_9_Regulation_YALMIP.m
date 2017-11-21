%% Example_9_Regulation_YALMIP.m
%
% Regulation of an LTI System using YALMIP
%
% Related Slides:         9-35 to 9-37
% Related Simulink Model: none
%
% Experiments:            none
%
% Remarks:                Requires YALMIP (yalmip.github.io)
%
% Model Predictive Control
%
% Jun.-Prof. Dr.-Ing. Daniel Görges
% Juniorprofessor for Electromobility
% Department of Electrical and Computer Engineering
% University of Kaiserslautern
% www.eit.uni-kl.de/en/jem/teaching/lectures/
%
% Please report any error to goerges@eit.uni-kl.de

%% Clear
clear;
close all;

%% Modeling of the LTI System
A = [1 1; 0 1];
B = [1; 0.5];
[nx nu] = size(B);

%% Definition of the Cost Function
Q = eye(nx);
R = eye(nu);

%% Definition of the Constraints
x_min = [-5; -5];
x_max = [5; 5];
u_min = -1;
u_max = 1;

%% Definition of the Model Predictive Controller
N = 5;
u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x = sdpvar(repmat(nx,1,N),repmat(1,1,N));
x_0 = sdpvar(nx,1);

constr = [];
cost = 0;
x{1} = x_0;
for i = 1:N
    x{i+1} = A*x{i}+B*u{i};
    cost = cost+x{i}'*Q*x{i}+u{i}*R*u{i};
    constr = [constr, x_min <= x{i} <= x_max, u_min <= u{i} <= u_max]; 
end;

%% Computation of the Optimal Input
optimize([constr, x_0 == [1; 1.5]],cost);
value(u{1})

%% Computation of the Closed-Loop Input and State Sequence
N_sim = 15;
x_sim{1} = [1; 1.5];
for k = 1:N_sim
    optimize([constr, x_0 == x_sim{k}],cost);
    u_sim{k} = value(u{1});
    x_sim{k+1} = A*x_sim{k}+B*u_sim{k};
end;

%% Visualization of the Closed-Loop Input and State Sequence
figure;
subplot(2,1,1)
plot(0:N_sim-1,cell2mat(u_sim)');
xlabel('k');
ylabel('u');
subplot(2,1,2)
plot(0:N_sim,cell2mat(x_sim)');
xlabel('k');
ylabel('x');