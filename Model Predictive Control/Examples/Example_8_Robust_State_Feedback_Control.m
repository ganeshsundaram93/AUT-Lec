%% Example_8_Robust_State_Feedback_Control.m
%
% Robust State Feedback Control
%
% Related Slides:         8-5 to 8-7, 8-19
% Related Simulink Model: none
%
% Experiments:            none
%
% Remarks:                Requires YALMIP (yalmip.github.io)
%                         Requires SeDuMi (sedumi.ie.lehigh.edu)
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

%% System with Polytopic Uncertainty
h = 0.5;                               % sampling period
alpha = [1/4 1/2];                     % uncertain parameter
beta = [1/16 1/4];                     % uncertain parameter
c = 2;                                 % stiffness constant
b = 1;                                 % damping constant

J = 0;                                 % counter, later number of vertices
for j = 1:2
    for i = 1:2
        J = J+1;
        A{J} = [1 h; -c*h*alpha(j) 1-b*h*alpha(j)];
        B{J} = [h^2/2*alpha(j); h*alpha(j)-b*h^2/2*beta(i)];
    end;
end;

[n m] = size(B{1});                    % determine system order

%% LMI Formulation
for j = 1:J
    Q{j} = sdpvar(n,n,'symmetric');    % initialize variables
end;
G = sdpvar(n,n);
Y = sdpvar(m,n);

constraints = [];                      % initialize constraints
for j = 1:J
    constraints = [constraints, Q{j} >= 0]; % note that strict inequalities are not reasonable in numerical optimization
    for i = 1:J
        constraints = [constraints, [G+G'-Q{j} G'*A{j}'+Y'*B{j}'; A{j}*G+B{j}*Y Q{i}] >= 0]; % note that strict inequalities are not reasonable in numerical optimization
    end;
end;

constraints

%% Solution of the LMI Problem
options = sdpsettings('solver','sedumi');
diagnostics = optimize(constraints,[],options)

check(constraints)

K = value(Y)*inv(value(G))

%% Simulation of the Closed-Loop System for the Vertices of the Polytope
N_sim = 20;

figure;
hold on;

for j = 1:4
    x{1} = [1; 0];
    for k = 1:N_sim
        u{k} = K*x{k};
        x{k+1} = A{j}*x{k}+B{j}*u{k};
    end;
    x_plot = cell2mat(x)';
    plot(0:N_sim,x_plot(:,1),'b');
    plot(0:N_sim,x_plot(:,2),'r');
end;

xlabel('k');
ylabel('x');
legend('x_1','x_2');

%% Simulation of the Closed-Loop System for Random Parameters alpha and beta
N_sim = 20;

figure;
hold on;

for i = 1:100
    x{1} = [1; 0];
    for k = 1:N_sim
        alpha(k) = (1/2-1/4)*rand+1/4;
        beta(k) = (1/4-1/16)*rand+1/16;
        A{k} = [1 h; -c*h*alpha(k) 1-b*h*alpha(k)];
        B{k} = [h^2/2*alpha(k); h*alpha(k)-b*h^2/2*beta(k)];
        u{k} = K*x{k};
        x{k+1} = A{k}*x{k}+B{k}*u{k};
    end;
    x_plot = cell2mat(x)';
    plot(0:N_sim,x_plot(:,1),'b');
    plot(0:N_sim,x_plot(:,2),'r');
end;

xlabel('k');
ylabel('x');
legend('x_1','x_2');