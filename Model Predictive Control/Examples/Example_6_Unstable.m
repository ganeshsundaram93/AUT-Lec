%% Example_6_Unstable.m
%
% Investigation of Optimal Cost Function
%
% Related Slides:         6-3 to 6-5
% Related Simulink Model: none
%
% Experiments:            Try different initial states (cf. line 116)
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

%% Parameters
A = [1.1 2; 0 0.95];        % system matrix
B = [0;0.0787];             % input matrix
C = [-1 1];                 % output matrix
N = 16;                     % prediction horizon
Q = C'*C;                   % state weighting matrix
R = 0.01;                   % input weighting matrix
P = dare(A,B,Q,R);          % terminal weighting matrix
[n m] = size(B);            % dimensions
u_min = -1;                 % input constraint
u_max = 1;                  % input constraint
x_min = [-1000; -1000];     % state constraint
x_max = [1000; 1000];       % state constraint

%% Predition Model
Phi = zeros(N*n,n);
Gamma = zeros(N*n,N*m);
for r = 1:N
    Phi((r-1)*n+1:r*n,1:n) = A^r;
    for c = 1:N
        if r >= c
            Gamma((r-1)*n+1:r*n,(c-1)*m+1:c*m) = A^(r-c)*B;
        end;
    end;
end;

%% Cost Function
Omega = zeros(N*n,N*n);
Psi = zeros(N*m,N*m);
for r = 1:N
    if r < N
        Omega((r-1)*n+1:r*n,(r-1)*n+1:r*n) = Q;
    else
        Omega((r-1)*n+1:r*n,(r-1)*n+1:r*n) = P;
    end;
    Psi((r-1)*m+1:r*m,(r-1)*m+1:r*m) = R;
end;
H = 2*(Psi+Gamma'*Omega*Gamma);
H = (H+H')/2;               % symmetrization
F = 2*Gamma'*Omega*Phi;

%% Constraint Model
for k = 1:N+1
    
    if k <= N
        M{k} = [zeros(m,n); zeros(m,n); -eye(n); eye(n)];
        [r_M c_M] = size(M{k});
        E{k} = [-eye(m); eye(m); zeros(n,m); zeros(n,m)];
        [r_E c_E] = size(E{k});
        b{k} = [-u_min; u_max; -x_min; x_max];
        [r_b c_b] = size(b{k});
    else
        M{k} = [-eye(n); eye(n)];
        [r_M c_M] = size(M{k});
        b{k} = [-x_min; x_max];
        [r_b c_b] = size(b{k});
    end;
    
    if k == 1
        D_cal((k-1)*r_M+1:k*r_M,1:c_M) = M{k};
        M_cal((k-1)*r_M+1:k*r_M,1:c_M) = zeros(r_M,c_M);
        E_cal((k-1)*r_E+1:k*r_E,(k-1)*c_E+1:k*c_E) = E{k};
        c((k-1)*r_b+1:k*r_b,1:c_b) = b{k};
    elseif k > 1 && k < N+1
        D_cal((k-1)*r_M+1:k*r_M,1:c_M) = zeros(r_M,c_M);
        M_cal((k-1)*r_M+1:k*r_M,(k-2)*c_M+1:(k-1)*c_M) = M{k};
        E_cal((k-1)*r_E+1:k*r_E,(k-1)*c_E+1:k*c_E) = E{k};
        c((k-1)*r_b+1:k*r_b,1:c_b) = b{k};
        r_M_old = r_M; c_M_old = c_M;
        r_E_old = r_E; c_E_old = c_E;
        r_b_old = r_b; c_b_old = c_b;
    else
        D_cal((k-1)*r_M_old+1:(k-1)*r_M_old+r_M,1:c_M) = zeros(r_M,c_M);
        M_cal((k-1)*r_M_old+1:(k-1)*r_M_old+r_M,(k-2)*c_M_old+1:(k-2)*c_M_old+c_M) = M{k};
        E_cal((k-1)*r_E_old+1:(k-1)*r_E_old+r_M,(k-2)*c_E_old+1:(k-2)*c_E_old+c_E) = zeros(r_M,c_E);
        c((k-1)*r_b_old+1:(k-1)*r_b_old+r_b,1:c_b) = b{k};
    end;
    
end;

J = M_cal*Gamma+E_cal;
W = -D_cal-M_cal*Phi;

%% Simulation
N_sim = 101;                                               % simulation steps
options = optimset('Algorithm','active-set');              % for older MATLAB
% options = optimset('Algorithm','interior-point-convex'); % for newer MATLAB

initial_state = 1;
if initial_state == 1
    x{1} = [0.5;-0.5];                                     % initial state
else
    x{1} = [0.8;-0.8];                                     % initial state
end;

for k = 1:N_sim
    [U fval(k) exitflag output lambda] = quadprog(H,F*x{k},J,c+W*x{k},[],[],[],[],[],options); % solve QP
    V(k) = x{k}'*Q*x{k}+U(1)'*R*U(1)+(A*x{k})'*Q*(A*x{k})+U(2)'*R*U(2)+(A^2*x{k})'*P*(A^2*x{k});
    u(k) = [eye(m,m) zeros(m,(N-1)*m)]*U;                                                   % get first element of the solution
    x{k+1} = A*x{k}+B*u(k);                                                                 % evaluate state equation for first element of the solution
    y(k) = C*x{k};                                                                          % evaluate output equation
end;

LineWidth = 1;
LineWidthAxes = 1;
FontName = 'Cambria';
FontSize = 16;

figure(1);

subplot(2,1,1);
hold on;
stairs(0:N_sim-1,u,'Marker','none','LineWidth',LineWidth,'Color','b');
set(gca,'LineWidth',LineWidthAxes);
set(gca,'FontName',FontName,'FontSize',FontSize);
set(gca,'XTick',[0:10:N_sim-1]);
xlabel('{\it k}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
ylabel('{\it u}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
axis([0 N_sim-1 -1.1 1.1]);

subplot(2,1,2);
hold on;
plot(0:N_sim-1,y,'Marker','none','LineWidth',LineWidth);
set(gca,'LineWidth',LineWidthAxes);
set(gca,'XTick',[0:10:N_sim-1]);
set(gca,'FontName',FontName,'FontSize',FontSize);
xlabel('{\it k}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
ylabel('{\it y}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
if initial_state == 1
    axis([0 N_sim-1 -5 5]);
else
    axis([0 N_sim-1 -200 50]);
end;

figure(2);
plot(0:N_sim-1,V,'Marker','none','LineWidth',LineWidth);
set(gca,'LineWidth',LineWidthAxes);
set(gca,'XTick',[0:10:N_sim-1]);
set(gca,'FontName',FontName,'FontSize',FontSize);
xlabel('{\it k}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');