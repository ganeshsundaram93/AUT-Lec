%% Example_6_Terminal_Constraints_Set.m
%
% Construction of the Terminal Constraint Set for Box Constraints
%
% Related Slides:         6-15 to 6-25
% Related Simulink Model: none
%
% Experiments:            Try with pause (cf. line 64)
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
R = 1;                      % input weighting matrix
[K P ~] = dlqr(A,B,Q,R);    % terminal weighting matrix and feedback matrix
K = -K;
[n m] = size(B);            % dimensions
u_min = -1;                 % input constraint
u_max = 1;                  % input constraint
x_min = [-1000; -1000];     % state constraint
x_max = [1000; 1000];       % state constraint

%% Terminal Constraint Set
LineWidth = 1;
LineWidthAxes = 1;
FontName = 'Cambria';
FontSize = 16;
cmap = colormap(lines(6));

x_1 = [-5:0.01:5];
for i = N:N+5
    M{i} = K*[A+B*K]^(i-N);
    figure;
    hold on;
    for j = N:i
        plot(x_1,(u_max-M{j}(1)*x_1)/M{j}(2),'LineWidth',LineWidth,'Color',cmap(j-N+1,:));
        plot(x_1,(u_min-M{j}(1)*x_1)/M{j}(2),'LineWidth',LineWidth,'Color',cmap(j-N+1,:));
        axis([-5 5 -0.8 0.8]);
    end;
    set(gca,'LineWidth',LineWidthAxes);
    set(gca,'FontName',FontName,'FontSize',FontSize);
    set(gca,'XTick',[-5:5]);
    set(gca,'YTick',[-0.8:0.2:0.8]);
    xlabel('{\it x}_1({\itk}+{\itN})','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
    ylabel('{\it x}_2({\itk}+{\itN})','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
    % pause
end;