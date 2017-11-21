%% Example_9_Tracking_Mass_Spring_Damper_System.m
%
% Tracking Control of a Mass-Spring-Damper System
%
% Related Slides:         9-23
% Related Simulink Model: none
%
% Experiments:            Investigate the influence of the state weighting matrix
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
Ts = 1;
sys = LTISystem('A',A,'B',B,'Ts',Ts)
x_0 = [0; 0];
sys.initialize(x_0)

%% Definition of the Cost Function
Q = diag([10000 1]);
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

%% Definition of a Time-Varying State Reference
sys.x.with('reference');
sys.x.reference = 'free';

%% Definition of the Model Predictive Controller
N = 5;
ctrl = MPCController(sys,N)

%% Definition of the Closed-Loop System
loop = ClosedLoop(ctrl,sys)

%% Computation of the Closed-Loop Input and State Sequence
N_sim = 50;
x_ref = 0.5*[zeros(1,10) ones(1,20) zeros(1,N_sim-10-20); zeros(1,N_sim)];
data = loop.simulate(x_0,N_sim,'x.reference',x_ref)

%% Visualization of the Closed-Loop Input and Input Sequence and the Reference Sequence
figure;
subplot(2,1,1);
plot(0:length(data.U)-1,data.U);
ylabel('u');
subplot(2,1,2);
hold on;
plot(0:length(data.X)-1,data.X);
stairs(0:length(x_ref(1,:))-1,x_ref(1,:),'b:');
ylabel('x');
legend('x_1','x_2','x_{1,ref}');
xlabel('t');