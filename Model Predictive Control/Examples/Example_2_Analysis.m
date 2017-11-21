%% Example_2_Analysis.m
%
% Analysis of the Continuous-Time System
%
% Related Slides:         2-7 to 2-9
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
C_c = [-k_s/m_s -b_s/m_s 0 b_s/m_s; 0 0 1 0];
D_c = [1/m_s; 0];
x_0 = [0; 0; 0; 0];

%% Analysis of the Continuous-Time System
lambda = eig(A_c)            % eigenvalues
[omega_0, zeta] = damp(A_c)  % natural frequencies (in rad/s) and damping
f = omega_0/(2*pi)           % natural frequencies (in Hz)