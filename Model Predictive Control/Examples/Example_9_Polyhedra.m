%% Example_9_Polyhedra.m
%
% Modeling of Polyhedra
%
% Related Slides:         9-6 to 9-8
% Related Simulink Model: none
%
% Experiments:            none
%
% Remarks:                Requires MPT (people.ee.ethz.ch/~mpt/3/)
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

%% Modeling of a Polyhedron with H-Representation using Inequalities
A = [eye(2); -eye(2)];
b = [1; 1; 1; 1];
P_1 = Polyhedron(A,b)
P_1.A
P_1.b
figure;
P_1.plot;

%% Modeling of a Polyhedron with H-Representation using Lower and Upper Bounds
P_2 = Polyhedron('lb',[-1 -1],'ub',[1 1])
figure;
P_2.plot;

%% Modeling of a Polyhedron with H-Representation using Equalities
Ae = eye(2);
be = [0; 0];
P_3 = Polyhedron('Ae',Ae,'be',be)
P_3.Ae
P_3.be
figure;
P_3.plot;

%% Modeling of a Polyhedron with V-Representation
P_4 = Polyhedron([-1 -1; -1 1; 1 -1; 1 1])
P_4.V
figure;
P_4.plot;

%% Modeling of a Polyhedron with H-Representation using Inequalities (cf. Slide 3-22)
A = [-1 0; 0 -1; 2 1; 0.5 1];
b = [0; 0; 2; 1];
P_5 = Polyhedron(A,b)
figure;
P_5.plot;
P_5.V