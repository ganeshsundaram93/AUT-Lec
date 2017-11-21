%% Example_5_RHC.m
%
% Receding-Horizon Control
%
% Related Slides:         5-8
% Related Simulink Model: none
%
% Experiments:            none
%
% Remarks:                Terminal cost and terminal constraint for ensuring stability and feasibility are not considered!                       
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
P = Q;                      % terminal weighting matrix
[n m] = size(B);            % dimensions
u_min = -1.5;               % input constraint
u_max = 1.5;                % input constraint
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
        D_tot((k-1)*r_M+1:k*r_M,1:c_M) = M{k};
        M_tot((k-1)*r_M+1:k*r_M,1:c_M) = zeros(r_M,c_M);
        E_tot((k-1)*r_E+1:k*r_E,(k-1)*c_E+1:k*c_E) = E{k};
        b_tot((k-1)*r_b+1:k*r_b,1:c_b) = b{k};
    elseif k > 1 && k < N+1
        D_tot((k-1)*r_M+1:k*r_M,1:c_M) = zeros(r_M,c_M);
        M_tot((k-1)*r_M+1:k*r_M,(k-2)*c_M+1:(k-1)*c_M) = M{k};
        E_tot((k-1)*r_E+1:k*r_E,(k-1)*c_E+1:k*c_E) = E{k};
        b_tot((k-1)*r_b+1:k*r_b,1:c_b) = b{k};
        r_M_old = r_M; c_M_old = c_M;
        r_E_old = r_E; c_E_old = c_E;
        r_b_old = r_b; c_b_old = c_b;
    else
        D_tot((k-1)*r_M_old+1:(k-1)*r_M_old+r_M,1:c_M) = zeros(r_M,c_M);
        M_tot((k-1)*r_M_old+1:(k-1)*r_M_old+r_M,(k-2)*c_M_old+1:(k-2)*c_M_old+c_M) = M{k};
        E_tot((k-1)*r_E_old+1:(k-1)*r_E_old+r_M,(k-2)*c_E_old+1:(k-2)*c_E_old+c_E) = zeros(r_M,c_E);
        b_tot((k-1)*r_b_old+1:(k-1)*r_b_old+r_b,1:c_b) = b{k};
    end;
    
end;

A_tot = M_tot*Gamma+E_tot;
W = -D_tot-M_tot*Phi;

%% Simulation
N_sim = 51;
options = optimset('Algorithm','active-set');              % for older MATLAB
% options = optimset('Algorithm','interior-point-convex'); % for newer MATLAB
x{1} = [0.5;-0.5];
x_sat{1} = x{1};
u_bound = 1.5;
K_LQR = dlqr(A,B,Q,R);
 
for k = 1:N_sim
    [U fval exitflag output lambda] = quadprog(H,F*x{k},A_tot,b_tot+W*x{k},[],[],[],[],[],options); % solve QP
    u(k) = [eye(m,m) zeros(m,(N-1)*m)]*U;                                                   % get first element of the solution
    x{k+1} = A*x{k}+B*u(k);                                                                 % evaluate state equation for first element of the solution
    y(k) = C*x{k};                                                                          % evaluate output equation
    
    u_sat(k) = -K_LQR*x_sat{k};
    if u_sat(k) < -u_bound
        u_sat(k) = -u_bound;
    elseif u_sat(k) > u_bound
        u_sat(k) = u_bound;
    end;
    x_sat{k+1} = A*x_sat{k}+B*u_sat(k);
    y_sat(k) = C*x_sat{k};
end;

LineWidth = 1;
LineWidthAxes = 1;
FontName = 'Cambria';
FontSize = 16;

subplot(2,1,1);
hold on;
stairs(0:N_sim-1,u,'Marker','none','LineWidth',LineWidth,'Color','b');
stairs(0:N_sim-1,u_sat,'Marker','none','LineWidth',LineWidth,'Color','r','LineStyle',':');
set(gca,'LineWidth',LineWidthAxes);
set(gca,'FontName',FontName,'FontSize',FontSize);
set(gca,'XTick',[0:10:N_sim-1]);
xlabel('{\it k}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
ylabel('{\it u}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
axis([0 N_sim-1 -10 10]);

subplot(2,1,2);
hold on;
plot(0:N_sim-1,y,'Marker','none','LineWidth',LineWidth);
plot(0:N_sim-1,y_sat,'Marker','none','LineWidth',LineWidth,'Color','r','LineStyle',':');
set(gca,'LineWidth',LineWidthAxes);
set(gca,'XTick',[0:10:N_sim-1]);
set(gca,'FontName',FontName,'FontSize',FontSize);
xlabel('{\it k}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
ylabel('{\it y}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
axis([0 N_sim-1 -5 5]);