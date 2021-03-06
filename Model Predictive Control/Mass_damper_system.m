clear;
close all;

%% Modeling of the LTI System
h = 0.5;
m = 4;
alpha = 1/m;
beta = 1/m^2;
c = 2;
b = 1;
A = [1 h; -c*h*alpha 1-b*h*alpha];
B = [h^2/2*alpha; h*alpha-b*h^2/2*beta];
C = [1 0];
sys = LTISystem('A',A,'B',B,'C',C,'Ts',h)
x_0 = [1; 0];
sys.initialize(x_0)

%% Definition of the Cost Function
Q = 100*eye(sys.nx); % identiy matrix with dimension of state vector
sys.x.penalty = QuadFunction(Q);
R = eye(sys.nu);% identiy matrix with dimension of input vector
sys.u.penalty = QuadFunction(R);

%% Definition of the Constraints (box)
sys.x.min = [-1; -0.5];
sys.x.max = [1; 0.5];
sys.u.min = -1.5;
sys.u.max = 1.5;

%% Definition of the Terminal Cost
P = sys.LQRPenalty;
sys.x.with('terminalPenalty'); %sys terminal penalty activaed 
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

%% Computation of the Closed-Loop Input, State, and Output Sequence and Cost
N_sim = 15;
data = loop.simulate(x_0,N_sim)

%% Comparison of the Closed-Loop and Optimal Input, State, and Output Sequence
figure;
subplot(2,1,1);
hold on;
plot(0:length(data.U)-1,data.U);
ylabel('u');
subplot(2,1,2);
hold on;
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
x{1} = [0 0];
for k = 1:N_sim
    u{k} = ctrl.evaluate(x{k});
        if k == 2
        w{k} = 0.1;
        else
        w{k} = 0;
    end;
    x{k+1} = A*x{k}+B*u{k}+w{k};
end;