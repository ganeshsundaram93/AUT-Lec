%% Example_8_Robust_Stability_Analysis.m
%
% Robust Stability Analysis
%
% Related Slides:         8-5 to 8-7, 8-17 to 8-18
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

n = length(A{1});                      % determine system order

%% LMI Formulation
for j = 1:J
    P{j} = sdpvar(n,n,'symmetric');    % initialize variables
end;

constraints = [];                      % initialize constraints
for j = 1:J
    constraints = [constraints, P{j} >= 0]; % note that strict inequalities are not reasonable in numerical optimization
    for i = 1:J
        constraints = [constraints, [P{j} A{j}'*P{i}; P{i}*A{j} P{i}] >= 0]; % note that strict inequalities are not reasonable in numerical optimization
        % constraints = [constraints, A{j}'*P{i}*A{j}-P{i} <= 0];
    end;
end;

constraints

%% Solution of the LMI Problem
options = sdpsettings('solver','sedumi');
diagnostics = optimize(constraints,[],options)

check(constraints)

for j = 1:J
    value(P{j})                       % show solutions P{j}
end;