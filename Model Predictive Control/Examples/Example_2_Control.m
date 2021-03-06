%% Example_2_Control.m
%
% State Feedback Control Design based on Pole Placement
%
% Related Slides:         2-33 to 2-35
% Related Simulink Model: Example_2_Control_Continuous_Time_Mdl.mdl
%                         Example_2_Control_Discrete_Time_Mdl.mdl
%
% Experiments:            Try non-zero initial state (cf. lines 45 and 46)
%                         Try sinusoidal bump and ramp-shaped bump (Manual Switch and Disturbance on/off set to 1 in Simulink models)
%                         Try different closed-loop poles (cf. line 64)
%                         Investigate states and input for the open-loop and closed-loop system (in Example_2_Control_Continuous_Time_Mdl.mdl)
%                         Investigate control effort (P = F*(z_s_dot-z_u_dot) in Example_2_Control_Continuous_Time_Mdl.mdl)
%                         Try selection of the sampling period based on "fast" and "slow" eigenvalues for non-zero initial state, sinusoidal bump and ramp-shaped bump (cf. lines 73 and 74, in Example_2_Control_Discrete_Time_Mdl.mdl)
%                         Try N_r = 4 and N_r = 10 (cf. line 75, in Example_2_Control_Discrete_Time_Mdl.mdl) for non-zero initial state, sinusoidal bump and ramp-shaped bump
%                         Investigate states in continuous time and discrete time (in Example_2_Control_Discrete_Time_Mdl.mdl)
%
% Remarks:                none
%
% Model Predictive Control
%
% Jun.-Prof. Dr.-Ing. Daniel G�rges
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
B_wc = [0; 0; -1; 0];
C_c = [-k_s/m_s -b_s/m_s 0 b_s/m_s; 0 0 1 0];
D_c = [1/m_s; 0];
% x_0 = [0; 0; 0; 0];
x_0 = [25*9.81/k_s; 0; 25*9.81/k_u; 0]; % additional mass of 25 kg

%% Disturbance
v = 40/3.6;        % speed

% sinusoidal bump
a_sin = 0.1;       % amplitude
l_sin = 5;         % length

% ramp-shaped bump
a_ramp = 0.05;     % amplitude
l_ramp = 0.05;     % length

%% Controllability of the Continuous-Time System
C_cctrb = ctrb(A_c,B_c) % controllability matrix
rank(C_cctrb)           % rank of the controllability matrix

%% Continuous-Time Control Design
lambda_tilde_c = [-37+70i;-37-70i;-4.5+8i;-4.5-8i];
K_c = place(A_c,B_c,lambda_tilde_c);
K_c = -K_c;        % Caution! Sign defined differently in MATLAB!

%% Analysis of the Continous-Time Closed-Loop System
lambda = eig(A_c+B_c*K_c)              % eigenvalues
[omega_0, zeta] = damp(A_c+B_c*K_c)    % natural frequencies (in rad/s) and damping
f = omega_0/(2*pi)                     % natural frequencies (in Hz)

%% Selection of the Sampling Period
T_r = omega_0(3)^(-1)*exp(acos(zeta(3))/tan(acos(zeta(3)))); % based on "slow" eigenvalues
% T_r = omega_0(1)^(-1)*exp(acos(zeta(1))/tan(acos(zeta(1)))); % based on "fast" eigenvalues
N_r = 4;
h = T_r/N_r

%% Discretization
sys_c = ss(A_c,B_c,C_c,D_c);
sys_dc = ss(A_c,B_wc,C_c,D_c);
sys = c2d(sys_c,h);
sys_d = c2d(sys_dc,h); % Caution! The disturbance is actually not subject to ZOH!
A = sys.a;
B_w = sys_d.b;
B = sys.b;
C = sys.c;
D = sys.d;

%% Controllability of the Discrete-Time System
C_ctrb = ctrb(A,B) % controllability matrix
rank(C_ctrb)       % rank of the controllability matrix

%% Discrete-Time Control Design
lambda_tilde = exp(lambda_tilde_c*h)
K = place(A,B,lambda_tilde);
K = -K;            % Caution! Sign defined differently in MATLAB!