%% Example_5_Detuned.m
%
% De-tuned Optimal Control
%
% Related Slides:         5-5 to 5-6
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
C = [-1 1];                 % output matrix
N = 2;                      % prediction horizon
Q = C'*C;                   % state weighting matrix
R = 0.01;                   % input weighting matrix
K_LQR = dlqr(A,B,Q,R);      % LQR
R = 100;
K_LQR_D = dlqr(A,B,Q,R);    % LQR (detuned)
[n m] = size(B);            % dimensions

%% Simulation
N_sim = 51;

x{1} = [0.5;-0.5];
x_D{1} = x{1};

for k = 1:N_sim
    
    u(k) = -K_LQR*x{k};
    x{k+1} = A*x{k}+B*u(k);
    y(k) = C*x{k};
    
    u_D(k) = -K_LQR_D*x_D{k};
    x_D{k+1} = A*x_D{k}+B*u_D(k);
    y_D(k) = C*x_D{k};
end;

fprintf('max_k(|u*(k)|) = %f\n',abs(max(u_D)));

LineWidth = 1;
LineWidthAxes = 1;
FontName = 'Cambria';
FontSize = 16;

subplot(2,1,1);
hold on;
stairs(0:N_sim-1,u,'Marker','none','LineWidth',LineWidth,'Color','b');
stairs(0:N_sim-1,u_D,'Marker','none','LineWidth',LineWidth,'Color','r','LineStyle',':');
set(gca,'LineWidth',LineWidthAxes);
set(gca,'FontName',FontName,'FontSize',FontSize);
set(gca,'XTick',[0:10:N_sim-1]);
xlabel('{\it k}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
ylabel('{\it u}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
axis([0 N_sim-1 -10 10]);

subplot(2,1,2);
hold on;
plot(0:N_sim-1,y,'Marker','none','LineWidth',LineWidth);
plot(0:N_sim-1,y_D,'Marker','none','LineWidth',LineWidth,'Color','r','LineStyle',':');
set(gca,'LineWidth',LineWidthAxes);
set(gca,'XTick',[0:10:N_sim-1]);
set(gca,'FontName',FontName,'FontSize',FontSize);
xlabel('{\it k}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
ylabel('{\it y}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
axis([0 N_sim-1 -5 5]);