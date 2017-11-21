%% Example_4_Stability_Performance.m
%
% Investigation of Stability and Performance
%
% Related Slides:         4-24 to 4-27
% Related Simulink Model: none
%
% Experiments:            Try different prediction horizons N
%                         Investigate the prediction accuracy
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
N = 2;                      % prediction horizon
Q = C'*C;                   % state weighting matrix
R = 0.01;                   % input weighting matrix
P = Q;                      % terminal weighting matrix
[n m] = size(B);            % dimensions

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
F = 2*Gamma'*Omega*Phi;

%% Analytical Solution
K_RHC = -[eye(m,m) zeros(m,(N-1)*m)]*inv(H)*F;
        
%% Simulation
N_sim = 8;

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

x{1} = [0.5;-0.5];

for k = 1:N_sim
    u(k) = K_RHC*x{k};
    x{k+1} = A*x{k}+B*u(k);
    y(k) = C*x{k};    
    U = -inv(H)*F*x{k};
    X = Phi*x{k}+Gamma*U;
    Y = C_diag*X;
    subplot(2,1,1);
    hold on;
    stairs(k-1:k+N-2,U,'LineStyle',':','Color',cmap(mod(k,7)+1,:),'Marker','x','LineWidth',LineWidth);
    stairs(k+N-2:k+N-1,[U(N) U(N)],'LineStyle',':','Color',cmap(mod(k,7)+1,:),'LineWidth',LineWidth); % to indicate ZOH!
    subplot(2,1,2);
    hold on;
    plot(k-1:k+N-1,[C*x{k}; Y],'LineStyle',':','Color',cmap(mod(k,7)+1,:),'Marker','x','LineWidth',LineWidth);
end;

figure(1);

subplot(2,1,1);
stairs(0:N_sim-1,u,'Marker','o','LineWidth',LineWidth);
set(gca,'LineWidth',LineWidthAxes);
set(gca,'FontName',FontName,'FontSize',FontSize);
set(gca,'XTick',[0:1:11]);
xlabel('{\it k}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
ylabel('{\it u}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
if N == 2
    axis([0 11 -80 80]);    
else
    axis([0 11 -10 10]);
end;

subplot(2,1,2);
plot(0:N_sim-1,y,'Marker','o','LineWidth',LineWidth);
set(gca,'LineWidth',LineWidthAxes);
set(gca,'XTick',[0:1:11]);
set(gca,'FontName',FontName,'FontSize',FontSize);
xlabel('{\it k}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
ylabel('{\it y}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
if N == 2
    axis([0 11 -15 15]);
else
    axis([0 11 -2 2]);
end;

figure(2);

sys = ss(A,B,C,0,1);
[y, k] = impulse(sys,10);
plot(k,y,'Marker','x','LineWidth',LineWidth);
set(gca,'LineWidth',LineWidthAxes);
set(gca,'XTick',[0:1:11]);
set(gca,'FontName',FontName,'FontSize',FontSize);
xlabel('{\it k}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
ylabel('{\it y}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
title('Impulse Response','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');