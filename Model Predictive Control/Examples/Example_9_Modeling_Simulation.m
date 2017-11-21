%% Example_9_Modeling_Simulation.m
%
% Modeling and Simulation of an LTI System
%
% Related Slides:         9-9 to 9-11
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

%% Modeling of an LTI System
A = [1 1; 0 1];
B = [1; 0.5];
C = [1 0];
D = 0;
Ts = 1; %samoling period
sys = LTISystem('A',A,'B',B,'C',C,'D',D,'Ts',Ts)

x_0 = [1; 1.5];
sys.initialize(x_0)

x = sys.getStates()
y = sys.output()

%% Simulation of the LTI System
u = 0.5;
x_next = sys.update(u)

U = [-2 -2 -1 0 1 2 2];
data = sys.simulate(U)

figure;
subplot(3,1,1);
plot(0:length(data.U)-1,data.U);
ylabel('u');
subplot(3,1,2);
plot(0:length(data.X)-1,data.X);
ylabel('x');
legend('x_1','x_2');
subplot(3,1,3);
plot(data.Y);
plot(0:length(data.Y)-1,data.Y);
xlabel('t');
ylabel('y');