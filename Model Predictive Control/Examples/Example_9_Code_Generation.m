%% Example_9_Code_Generation.m
%
% Export of a PWA State Feedback Controller to MATLAB and C Code
%
% Related Slides:         9-30 to 9-31
% Related Simulink Model: Example_9_Code_Generation_Mdl.mdl
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
x_0 = [1; 1.5];
sys.initialize(x_0)

%% Definition of the Cost Function
Q = eye(sys.nx);
sys.x.penalty = QuadFunction(Q);
R = eye(sys.nu);
sys.u.penalty = QuadFunction(R);

%% Definition of the Constraints
sys.x.min = [-5; -5];
sys.x.max = [5; 5];
sys.u.min = -1;
sys.u.max = 1;

%% Definition of the Terminal Cost
P = sys.LQRPenalty;
sys.x.with('terminalPenalty');
sys.x.terminalPenalty = P;

%% Definition of the Terminal Constraint Set
X_N = sys.LQRSet;
sys.x.with('terminalSet');
sys.x.terminalSet = X_N;

%% Definition of the Model Predictive Controller
ctrl = MPCController(sys,5)

%% Generation of an Explicit Model Predictive Controller
expctrl = ctrl.toExplicit()

%% Export of the PWA State Feedback Controller to MATLAB
expctrl.optimizer.toMatlab('mycontroller.m','primal','obj');

%% Computation of the Optimal Input Sequence and Region
[U, region] = mycontroller(x_0)

%% Reduction of the PWA State Feedack Controller
expctrl.optimizer.trimFunction('primal',ctrl.model.nu);
expctrl.optimizer.toMatlab('mycontroller_reduced.m','primal','obj');

%% Computation of the Closed-Loop Input, State, and Output Sequence
N_sim = 15;
x{1} = x_0;
for k = 1:N_sim
    u{k} = mycontroller_reduced(x{k});
    x{k+1} = A*x{k}+B*u{k};
    y{k} = C*x{k}+D*u{k}
end;

%% Visualization of the Closed-Loop Input, State, and Output Sequence
figure;
subplot(3,1,1);
hold on;
plot(0:N_sim-1,cell2mat(u)');
ylabel('u');
subplot(3,1,2);
hold on;
plot(0:N_sim,cell2mat(x)');
ylabel('x');
legend('x_1','x_2');
subplot(3,1,3);
plot(0:N_sim-1,cell2mat(y)');
hold on;
ylabel('y');
xlabel('t');

%% Export of the PWA State Feedback Controller to C
expctrl.exportToC('mycontroller');