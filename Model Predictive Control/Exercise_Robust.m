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

[n m] = size(B{1});                      % determine system order

%% 
for j = 1:J
    P{j} = sdpvar(n,n,'symmetric'); % initialize variables
    
end;
    G = sdpvar(n,n);
    Y = sdpvar(m,n);

constraints = [];                      % initialize constraints
for j = 1:J
    constraints = [constraints P{j} >= 0]; %positive definite note that strict inequalities are not reasonable in numerical optimization
    for i = 1:J
        constraints = [constraints [G+G'-P{j} G'*A{j}'+Y'*B{j}';A{j}*G+B{j}*Y P{i}] >= 0];
        % constraints = [constraints, A{j}'*P{i}*A{j}-P{i} <= 0];
    end;
end;

constraints

% Solution of the LMI Problem
options = sdpsettings('solver','sedumi');
diagnostics = optimize(constraints,[],options);

checkset(constraints)

for j = 1:J
    double(P{j})                       % show solutions P{j}
end;

K = value(Y)*inv(value(G))

%% Simulation of closed loop sysytem for the vertices of prototype
N_sim = 20;

figure;
hold on;

for j = 1:4
    x{1} = [1;0];
    for k = 1:N_sim
        u{k} = K*x{k};
        x{k+1} = A{k}*x{k}+B{k}*u{k};
    end;
    
