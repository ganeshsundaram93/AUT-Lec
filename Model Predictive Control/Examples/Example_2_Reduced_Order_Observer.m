%% Example_2_Reduced_Order_Observer.m
%
% Reduced-Order Observer Design based on Pole Placement
%
% Related Slides:         2-44 to 2-46
% Related Simulink Model: Example_2_Reduced_Order_Observer_Mdl.mdl
%
% Experiments:            Try non-zero initial state (cf. lines 44 and 45)
%                         Try sinusoidal bump and ramp-shaped bump (Manual Switch and Disturbance on/off set to 1 in Simulink model)
%                         Try different observer poles (cf. line 68, lambda_tilde_c = real(lambda)*2...6+i*imag(lambda))
%                         Try selection of the sampling period based on "fast" and "slow" eigenvalues for non-zero initial state, sinusoidal bump and ramp-shaped bump (cf. lines 71 and 72)
%                         Try N_r = 4 and N_r = 10 (cf. line 73) for non-zero initial state, sinusoidal bump and ramp-shaped bump
%                         Investigate states and esimated states
%                         Why are the tire deflection z_u-z_r and the unsprung mass velocity z_u_dot not estimated very well?
%
% Remarks:                The reduced-order observer is implemented in Example_2_Reduced_Order_Observer_Mdl.mdl using the substitution described on Slide 2-46 (marked red)
%                         Why should the observer poles not be selected too "fast" if the reduced-order observer is implemented in this way?
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
C_c = [1 0 0 0];                        % suspension deflection x_1 = z_s-z_u as output
D_c = 0;
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
lambda_tilde_c = real(lambda)*2+i*imag(lambda);  % desired observer poles in Laplace domain
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

%% Reduced-Order Observer Design
A_11 = A(1,1);
A_12 = A(1,2:4);
A_21 = A(2:4,1);
A_22 = A(2:4,2:4);
B_1 = B(1,1);
B_2 = B(2:4,1);

O_obsv = obsv(A_22,A_12)     % observability matrix
rank(O_obsv)                 % rank of the observability matrix

temp = exp(lambda_tilde_c*h);
lambda_tilde = [temp(1:2); real(temp(3))]         % desired observer poles in z-domain
L_r = place(A_22',A_12',lambda_tilde);
L_r = L_r';
x_0_red = [0; 0; 0];