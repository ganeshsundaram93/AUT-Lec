%% Example_7_Terminal_Constraints_Set.m
%
% Preview Control for Disturbance Rejection
%
% Related Slides:         7-26 to 7-31
% Related Simulink Model: none
%
% Experiments:            none
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
B_w = B;                    % disturbance input matrix
C = [-1 1];                 % output matrix
C_w = 0;
N = 5;                      % prediction horizon
Q = C'*C;                   % state weighting matrix
R = 0.01;                   % input weighting matrix
P = dare(A,B,Q,R);          % terminal weighting matrix
[n m] = size(B);            % dimensions

%% Predition Model
Phi = zeros(N*n,n);
Gamma = zeros(N*n,N*m);
Gamma_w = zeros(N*n,N*m);
for r = 1:N
    Phi((r-1)*n+1:r*n,1:n) = A^r;
    for c = 1:N
        if r >= c
            Gamma((r-1)*n+1:r*n,(c-1)*m+1:c*m) = A^(r-c)*B;
            Gamma_w((r-1)*n+1:r*n,(c-1)*m+1:c*m) = A^(r-c)*B_w;
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
F = 2*Gamma'*Omega*Phi;
F_w = 2*Gamma'*Omega*Gamma_w;

%% Analytical Solution
K_RHC = -[eye(m,m) zeros(m,(N-1)*m)]*inv(H)*F;
K_wRHC = -[eye(m,m) zeros(m,(N-1)*m)]*inv(H)*F_w;
        
%% Simulation
N_sim = 31;

cmap = colormap(lines(7));
LineWidth = 1;
LineWidthAxes = 1;
Location = 'SouthEast';
FontName = 'Cambria';
FontSize = 16;

C_diag = [];
for r = 1:N
    C_diag = blkdiag(C_diag,C);
end;

x{1} = [0;0];
x_w{1} = x{1};
W = 2*[zeros(10,1); ones(10,1); zeros(N_sim-20+N,1)];

for k = 1:N_sim
    u(k) = K_RHC*x{k};
    x{k+1} = A*x{k}+B*u(k)+B_w*W(k);
    y(k) = C*x{k}+C_w*W(k);    
    u_w(k) = K_RHC*x_w{k}+K_wRHC*W(k:k+N-1,1);
    x_w{k+1} = A*x_w{k}+B*u_w(k)+B_w*W(k);
    y_w(k) = C*x_w{k}+C_w*W(k);
end;

figure(1);

subplot(3,1,1);
hold on;
stairs(0:length(W)-1,W,'Marker','none','LineWidth',LineWidth,'Color','b');
set(gca,'LineWidth',LineWidthAxes);
set(gca,'FontName',FontName,'FontSize',FontSize);
set(gca,'XTick',[0:2:N_sim]);
set(gca,'YTick',[0:2:2]);
xlabel('{\it k}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
ylabel('{\it w}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
axis([0 N_sim-1 -0.5 2.5]);

subplot(3,1,2);
hold on;
stairs(0:N_sim-1,u_w,'Marker','none','LineWidth',LineWidth,'Color','b');
stairs(0:N_sim-1,u,'Marker','none','LineWidth',LineWidth,'Color','r','LineStyle',':');
set(gca,'LineWidth',LineWidthAxes);
set(gca,'FontName',FontName,'FontSize',FontSize);
set(gca,'XTick',[0:2:N_sim]);
set(gca,'YTick',[-4:4:4]);
xlabel('{\it k}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
ylabel('{\it u}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
axis([0 N_sim-1 -4 4]);

subplot(3,1,3);
hold on;
plot(0:N_sim-1,y_w,'Marker','none','LineWidth',LineWidth,'Color','b');
plot(0:N_sim-1,y,'Marker','none','LineWidth',LineWidth,'Color','r','LineStyle',':');
set(gca,'LineWidth',LineWidthAxes);
set(gca,'XTick',[0:2:N_sim]);
set(gca,'FontName',FontName,'FontSize',FontSize);
xlabel('{\it k}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
ylabel('{\it y}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
axis([0 N_sim-1 -1 1]);