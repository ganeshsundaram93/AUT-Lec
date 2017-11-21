%% Example_9_Regulation.m
%
% Regulation of an LTI System
%
% Related Slides:         9-12 to 9-18
% Related Simulink Model: none
%
% Experiments:            Investigate the influence of the prediction horizon
%                         Investigate the influence of the terminal cost and constraint set
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
x_0 = [1; 1.5];
sys.initialize(x_0)

%% Definition of the Cost Function
Q = eye(sys.nx); % identiy matrix with dimension of state vector
sys.x.penalty = QuadFunction(Q);
R = eye(sys.nu);% identiy matrix with dimension of input vector
sys.u.penalty = QuadFunction(R);

%% Definition of the Constraints (box)
sys.x.min = [-5; -5];
sys.x.max = [5; 5];
sys.u.min = -1;
sys.u.max = 1;

%% Definition of the Terminal Cost
P = sys.LQRPenalty;
sys.x.with('terminalPenalty'); %sys terminal penalty activaed 
sys.x.terminalPenalty = P;

%% Definition of the Terminal Constraint Set
X_N = sys.LQRSet;
sys.x.with('terminalSet');
sys.x.terminalSet = X_N;
X_N.plot;

%% Definition of the Model Predictive Controller
N = 5;
ctrl = MPCController(sys,N)

%% Computation of the Optimal Input
u = ctrl.evaluate(x_0)

%% Computation of the Optimal Input, State, and Output Sequence and Cost
[u feasible openloop] = ctrl.evaluate(x_0)

%% Visualization of the Optimal Input, State, and Output Sequence
figure;
ctrl.model.plot();

%% Definition of the Closed-Loop System
loop = ClosedLoop(ctrl,sys)

%% Computation of the Closed-Loop Input, State, and Output Sequence and Cost
N_sim = 15;
data = loop.simulate(x_0,N_sim)

%% Comparison of the Closed-Loop and Optimal Input, State, and Output Sequence
figure;
subplot(3,1,1);
hold on;
plot(0:length(data.U)-1,data.U);
plot(0:length(openloop.U)-1,openloop.U,':');
ylabel('u');
legend('closed loop','optimal');
subplot(3,1,2);
hold on;
plot(0:length(data.X)-1,data.X);
plot(0:length(openloop.X)-1,openloop.X,':');
ylabel('x');
legend('x_1','x_2');
subplot(3,1,3);
hold on;
plot(0:length(data.Y)-1,data.Y);
plot(0:length(openloop.Y)-1,openloop.Y,':');
ylabel('y');
xlabel('t');

%% Generation of an Explicit Model Predictive Controller
expctrl = ctrl.toExplicit()

%% Visualization of a PWA State Feedback Controller
figure;
expctrl.feedback.fplot();

%% Visualization of the Optimal Cost Function
figure;
expctrl.cost.fplot();

%% Visualization of the Regions
figure;
expctrl.partition.plot();

%% Visualization of the Closed-Loop State Trajectory
figure;
expctrl.clicksim();