%% Example_2_Disturbance_Estimation.m
%
% Disturbance Estimation
%
% Related Slides:         2-57 to 2-59
% Related Simulink Model: Example_2_Disturbance_Estimation_Mdl.mdl
%
% Experiments:            Try non-zero initial state (cf. lines 43 and 44)
%                         Try selection of the sampling period based on "fast" and "slow" eigenvalues for non-zero initial state, sinusoidal bump and ramp-shaped bump (cf. lines 71 and 72)
%                         Try N_r = 4 and N_r = 10 (cf. line 73) for non-zero initial state, sinusoidal bump and ramp-shaped bump
%                         Try without and with disturbance compensation (Disturbance Compensation on/off set to 0 or 1 in Simulink model)
%                         Try without and with state feedback control (State Feedback Control on/off set to 0 or 1 in Simulink model)
%                         Investigate the disturbance w_hat and the estimated disturbance w_hat
%                         Investigate the output y
%
% Remarks:                Note that B and B_w must be distinguished in (2.25) for the active suspension system! How?
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
C_c = [-k_s/m_s -b_s/m_s 0 b_s/m_s];
D_c = 1/m_s;
x_0 = [0; 0; 0; 0];
% x_0 = [25*9.81/k_s; 0; 25*9.81/k_u; 0]; % additional mass of 25 kg

%% Analysis of the Continuous-Time System
lambda = eig(A_c)            % eigenvalues
[omega_0, zeta] = damp(A_c)  % natural frequencies (in rad/s) and damping
f = omega_0/(2*pi)           % natural frequencies (in Hz)

%% Selection of the Sampling Period
lambda_tilde_c = real(lambda)*6+i*imag(lambda);  % desired observer poles in Laplace domain
omega_0_tilde = abs(lambda_tilde_c);
zeta_tilde = -cos(angle(lambda_tilde_c));
% T_r = omega_0_tilde(3)^(-1)*exp(acos(zeta_tilde(3))/tan(acos(zeta_tilde(3)))); % based on "slow" eigenvalues
T_r = omega_0_tilde(1)^(-1)*exp(acos(zeta_tilde(1))/tan(acos(zeta_tilde(1)))); % based on "fast" eigenvalues
N_r = 4;
h = T_r/N_r

%% Disturbance Model
v = 40/3.6;        % speed
a = 0.1;           % amplitude
l = 5;             % length
T = l/v;           % period
omega = 2*pi/T;
A_cw = [0 1; -omega^2 0];
C_cw = [1 0];

%% Discretization
sys_c = ss(A_c,B_c,C_c,D_c);
sys_dc = ss(A_c,B_wc,C_c,D_c);
sys_wc = ss(A_cw,[0;0],C_cw,0);
sys = c2d(sys_c,h);
sys_d = c2d(sys_dc,h); % Caution! The disturbance is actually not subject to ZOH!
sys_w = c2d(sys_wc,h);
A = sys.a;
A_w = sys_w.a;
B = sys.b;
B_w = sys_d.b;
C = sys.c;
C_w = sys_w.c;
D = sys.d;

%% Controllability of the Discrete-Time System
C_ctrb = ctrb(A,B) % controllability matrix
rank(C_ctrb)       % rank of the controllability matrix

%% Discrete-Time Control Design
lambda_tilde = exp(lambda_tilde_c*h)
K = place(A,B,lambda_tilde);
K = -K;

%% Augmented Discrete-Time System
[n m] = size(B);
[p n] = size(C);
n_w = length(A_w);
A_aug = [A B_w*C_w; zeros(n_w,n) A_w]; % Caution! B_d must be considered here!
B_aug = [B; zeros(n_w,m)];
B_w_aug = [B_w; zeros(n_w,m)];
C_aug = [C zeros(p,n_w)];
D_aug = D;

%% Observability of the Augmented Discrete-Time System
O_aug_obsv = obsv(A_aug,C_aug)    % observability matrix
rank(O_aug_obsv)                  % rank of the observability matrix

%% Disturbance Observer Design
lambda_tilde = [exp(lambda_tilde_c*h); 0.8; 0.85]; % desired observer poles in z-domain
L = place(A_aug',C_aug',lambda_tilde);
L = L';
x_0_obsv = [0; 0; 0; 0; 0; 0];