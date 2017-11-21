%% Example_9_Tracking_Delta_Input_Formulation.m
%
% Tracking of a Time-Vayring Output Reference using the Delta Input Formulation
%
% Related Slides:         9-21
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

%% Modeling of the LTI System
A = [1 1; 0 1];
B = [1; 0.5];
C = [1 0];
D = 0;
Ts = 1;
sys = LTISystem('A',A,'B',B,'C',C,'D',D,'Ts',Ts)
x_0 = [0; 0];
sys.initialize(x_0)

%% Definition of the Cost Function
Q = eye(sys.ny);
sys.y.penalty = QuadFunction(Q);
sys.u.with('deltaPenalty');
R = eye(sys.nu);
sys.u.deltaPenalty = QuadFunction(R);

%% Definition of the Constraints
sys.x.min = [-5; -5];
sys.x.max = [5; 5];
sys.u.min = -1;
sys.u.max = 1;

%% Definition of a Time-Varying Output Reference
sys.y.with('reference');
sys.y.reference = 'free';

%% Definition of the Model Predictive Controller
ctrl = MPCController(sys,5)

%% Definition of the Closed-Loop System
loop = ClosedLoop(ctrl,sys)

%% Computation of the Closed-Loop Input, State, and Output Sequence and Cost
N_sim = 30;
u_0 = 0;
y_ref = [ones(1,10) 2*ones(1,10) 3*ones(1,10)];
data = loop.simulate(x_0,N_sim,'u.previous',u_0,'y.reference',y_ref)

%% Visualization of the Closed-Loop Output Sequence and the Reference Sequence
figure;
hold on;
plot(0:N_sim-1,data.Y,'b');
stairs(0:N_sim-1,y_ref,'b:');
legend('y','y_{ref}');
xlabel('t');

%% Generation of an Explicit Model Predictive Controller
expctrl = ctrl.toExplicit()

%% Visualization of the Regions
figure;
expctrl.partition.plot();