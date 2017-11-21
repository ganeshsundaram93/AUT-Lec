%% Example_4_Active_Suspension_System.m
%
% Selection of the Weighting Matrices based on Bryson's Rule
%
% Related Slides:         4-5
% Related Simulink Model: Example_4_Active_Suspension_System_Mdl.mdl
%
% Experiments:            Investigate states and input for the open-loop and closed-loop system
%                         Try to modify the state and input weighting matrices (cf. lines 96 to 98)
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

%% Quarter-Car Parameters [PSD+08]
m_s = 315;         % sprung mass
k_s = 29500;       % suspension stiffness
b_s = 1500;        % suspension damping
m_u = 37.5;        % unsprung mass
k_u = 210000;      % tire stiffness

%% State-Space Model
A_c = [0 1 0 -1; -k_s/m_s -b_s/m_s 0 b_s/m_s; 0 0 0 1; k_s/m_u b_s/m_u -k_u/m_u -b_s/m_u];
B_c = [0; 1/m_s; 0; -1/m_u];
B_dc = [0; 0; -1; 0];
C_c = [-k_s/m_s -b_s/m_s 0 b_s/m_s; 0 0 1 0];
D_c = [1/m_s; 0];
x_0 = [0; 0; 0; 0];
% x_0 = [25*9.81/k_s; 0; 25*9.81/k_u; 0]; % additional mass of 25 kg

%% Disturbance
v = 40/3.6;        % speed

% sinusoidal bump
a_sin = 0.1;       % amplitude
l_sin = 5;         % length

% ramp bump
a_ramp = 0.05;     % amplitude
l_ramp = 0.05;     % length

%% Controllability of the Continuous-Time System
C_cctrb = ctrb(A_c,B_c) % controllability matrix
rank(C_cctrb)           % rank of the controllability matrix

%% Continuous-Time Control Design
lambda_tilde_c = [-37+70i;-37-70i;-4.5+8i;-4.5-8i]; % zeta = 0.45
K_c = place(A_c,B_c,lambda_tilde_c);
K_c = -K_c;

%% Analysis of the Continous-Time Closed-Loop System
lambda = eig(A_c+B_c*K_c)              % eigenvalues
[omega_0, zeta] = damp(A_c+B_c*K_c)    % natural frequencies (in rad/s) and damping
f = omega_0/(2*pi)                     % natural frequencies (in Hz)

%% Selection of the Sampling Period
T_r = omega_0(3)^(-1)*exp(acos(zeta(3))/tan(acos(zeta(3)))); % based on "slow" eigenvalues
% T_r = omega_0(1)^(-1)*exp(acos(zeta(1))/tan(acos(zeta(1)))); % based on "fast" eigenvalues
N_r = 10;
h = T_r/N_r

%% Discretization
sys_c = ss(A_c,B_c,C_c,D_c);
sys_dc = ss(A_c,B_dc,C_c,D_c);
sys = c2d(sys_c,h);
sys_d = c2d(sys_dc,h); % Caution! The disturbance is actually not subject to ZOH!
A = sys.a;
B_d = sys_d.b;
B = sys.b;
C = sys.c;
D = sys.d;

%% Controllability of the Discrete-Time System
C_ctrb = ctrb(A,B) % controllability matrix
rank(C_ctrb)       % rank of the controllability matrix

%% Simulation
K = [0 0 0 0];
sim('Example_4_Active_Suspension_System_Mdl');

%% Discrete-Time Control Design
q = 1./max(min(x).^2,max(x).^2);  % from state trajectory of the open-loop system resulting e.g. for sinusiodal bump
r = 1/(1000*1000);                % e.g. from datasheet of the force actuator

% q(2) = q(2)*1e1;                % Sprung mass velocity (i.e. ride comfort) more important?
% q(3) = q(3)*1e2;                % Tire deflection (i.e. handling performance) even more important?
% r = r*1e-1;                     % Control energy less important?

Q = diag(q);
R = diag(r);

K = dlqr(A,B,Q,R);
K = -K;                           % Caution! Sign defined differently in MATLAB!