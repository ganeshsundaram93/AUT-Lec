%% Example_4_Introduction.m
%
% Construction of the Prediction Model, Cost Function, Analytical Solution, and Receding Horizon Controller
%
% Related Slides:         4-11 to 4-13, 4-22
% Related Simulink Model: none
%
% Experiments:            none
%
% Remarks:                none
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

%% Parameters
A = [1.1 2; 0 0.95];        % system matrix
B = [0;0.0787];             % input matrix
C = [-1 1];                 % output matrix
Q = C'*C;                   % state weighting matrix
P = Q;                      % terminal weighting matrix
R = 0.01;                   % input weighting matrix
N = 4;                      % prediction horizon
[n m] = size(B);            % dimensions

%% Predition Model
Phi = zeros(N*n,n);
Gamma = zeros(N*n,N*m);
for r = 1:N
    Phi((r-1)*n+1:r*n,1:n) = A^r;
    for c = 1:N
        if r >= c
            Gamma((r-1)*n+1:r*n,(c-1)*m+1:c*m) = A^(r-c)*B;
        end;
    end;
end;

%% Cost Function
Omega = zeros(N*n,N*n);
Psi = zeros(N*m,N*m);
for r = 1:N
    if r < N
        Omega((r-1)*n+1:r*n,(r-1)*n+1:r*n) = Q;
    else
        Omega((r-1)*n+1:r*n,(r-1)*n+1:r*n) = P;
    end;
    Psi((r-1)*m+1:r*m,(r-1)*m+1:r*m) = R;
end;
H = 2*(Psi+Gamma'*Omega*Gamma);
F = 2*Gamma'*Omega*Phi;

%% Analytical Solution
M = -inv(H)*F;

%% Receding Horizon Controller
K_RHC = -[eye(m,m) zeros(m,(N-1)*m)]*M;