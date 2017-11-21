%% Example_2_Prediciton_Observer.m
%
% Prediction Observer Design based on Pole Placement
%
% Related Slides:         2-37 to 2-40
% Related Simulink Model: Example_2_Prediciton_Observer_Mdl.mdl
%
% Experiments:            Try non-zero initial state (cf. lines 42 and 43)
%                         Try sinusoidal bump and ramp-shaped bump (Manual Switch and Disturbance on/off set to 1 in Simulink model)
%                         Try different observer poles (cf. line 66, lambda_tilde_c = real(lambda)*2...6+i*imag(lambda))
%                         Try selection of the sampling period based on "fast" and "slow" eigenvalues for non-zero initial state, sinusoidal bump and ramp-shaped bump (cf. lines 69 and 70)
%                         Try N_r = 4 and N_r = 10 (cf. line 71) for non-zero initial state, sinusoidal bump and ramp-shaped bump
%                         Investigate states and esimated states
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

%% Quarter-Car Parameters
m_s = 315;         % sprung mass
k_s = 29500;       % suspension stiffness
b_s = 1500;        % suspension damping
m_u = 37.5;        % unsprung mass
k_u = 210000;      % tire stiffness

%% State-Space Model
A_c = [0 1 0 -1; -k_s/m_s -b_s/m_s 0 b_s/m_s; 0 0 0 1; k_s/m_u b_s/m_u -k_u/m_u -b_s/m_u];
B_c = [0; 1/m_s; 0; -1/m_u];
B_wc = [0; 0; -1; 0];
C_c = [-k_s/m_s -b_s/m_s 0 b_s/m_s];    % sprung mass acceleration x_2_dot = z_s_dot_dot as output
D_c = 1/m_s;
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

%% Observability of the Continuous-Time System
O_cobsv = obsv(A_c,C_c) % observability matrix
rank(O_cobsv)           % rank of the observability matrix

%% Analysis of the Continuous-Time System
lambda = eig(A_c)            % eigenvalues
[omega_0, zeta] = damp(A_c)  % natural frequencies (in rad/s) and damping
f = omega_0/(2*pi)           % natural frequencies (in Hz)

%% Selection of the Sampling Period
lambda_tilde_c = real(lambda)*6+i*imag(lambda);  % desired observer poles in Laplace domain
omega_0_tilde = abs(lambda_tilde_c);
zeta_tilde = -cos(angle(lambda_tilde_c));
T_r = omega_0_tilde(3)^(-1)*exp(acos(zeta_tilde(3))/tan(acos(zeta_tilde(3)))); % based on "slow" eigenvalues
% T_r = omega_0_tilde(1)^(-1)*exp(acos(zeta_tilde(1))/tan(acos(zeta_tilde(1)))); % based on "fast" eigenvalues
N_r = 10;
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

%% Observability of the Discrete-Time System
O_obsv = obsv(A,C)      % observability matrix
rank(O_obsv)            % rank of the observability matrix

%% Prediction Observer Design
lambda_tilde = exp(lambda_tilde_c*h)             % desired observer poles in z-domain
L = place(A',C',lambda_tilde);
L = L';
x_0_obsv = [0; 0; 0; 0];