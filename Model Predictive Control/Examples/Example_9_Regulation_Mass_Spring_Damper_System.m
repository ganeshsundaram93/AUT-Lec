%% Example_9_Regulation_Mass_Spring_Damper_System.m
%
% Regulation of a Mass-Spring-Damper System
%
% Related Slides:         9-19 to 9-20
% Related Simulink Model: Example_9_Regulation_Mass_Spring_Damper_System_Mdl.mdl
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

%% Modeling of the Mass-Spring-Damper System
h = 0.5;                               % sampling period
m = 4;                                 % mass
alpha = 1/m;                           % parameter
beta = 1/m^2;                          % parameter
c = 2;                                 % stiffness constant
b = 1;                                 % damping constant
A = [1 h; -c*h*alpha 1-b*h*alpha];
B = [h^2/2*alpha; h*alpha-b*h^2/2*beta];
sys = LTISystem('A',A,'B',B,'Ts',h)
x_0 = [1; 0];
sys.initialize(x_0)

%% Definition of the Cost Function
Q = 100*eye(sys.nx);
sys.x.penalty = QuadFunction(Q);
R = eye(sys.nu);
sys.u.penalty = QuadFunction(R);

%% Definition of the Constraints
sys.x.min = [-1; -0.5];
sys.x.max = [1; 0.5];
sys.u.min = -1.5;
sys.u.max = 1.5;

%% Definition of the Terminal Cost
P = sys.LQRPenalty;
sys.x.with('terminalPenalty');
sys.x.terminalPenalty = P;

%% Definition of the Terminal Constraint Set
X_N = sys.LQRSet;
sys.x.with('terminalSet');
sys.x.terminalSet = X_N;

%% Definition of the Model Predictive Controller
N = 5;
ctrl = MPCController(sys,N)

%% Definition of the Closed-Loop System
loop = ClosedLoop(ctrl,sys)

%% Computation of the Closed-Loop Input and State Sequence
N_sim = 20;
data = loop.simulate(x_0,N_sim)

%% Visualization of the Closed-Loop Input and State Sequence
figure;
subplot(2,1,1);
plot(0:length(data.U)-1,data.U);
ylabel('u');
subplot(2,1,2);
plot(0:length(data.X)-1,data.X);
ylabel('x');
legend('x_1','x_2');
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

%% Computation of the Closed-Loop Input and State Sequence under the Disturbance
x{1} = [0; 0];
for k = 1:N_sim
    u{k} = ctrl.evaluate(x{k});
    if k == 2
        w{k} = 0.1;
    else
        w{k} = 0;
    end;
    x{k+1} = A*x{k}+B*u{k}+w{k};
end;

%% Visualization of the Closed-Loop Input and State Sequence under the Disturbance
figure;
subplot(2,1,1)
plot(0:N_sim-1,cell2mat(u)');
xlabel('k');
ylabel('u');
subplot(2,1,2)
plot(0:N_sim,cell2mat(x)');
xlabel('k');
ylabel('x');
legend('x_1','x_2');