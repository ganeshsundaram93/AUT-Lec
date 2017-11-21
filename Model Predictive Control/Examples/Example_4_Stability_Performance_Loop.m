%% Example_4_Stability_Performance_Loop.m
%
% Investigation of Stability for Various Input Weighting Matrices and Prediction Horizons
%
% Related Slides:         4-23
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
Q = C'*C;                   % state weighting matrix
P = Q;                      % terminal weighting matrix
[n m] = size(B);            % dimensions

N_range = 2:10;             % range for prediction horizon
R_range = 1:1:100;          % range for input weighting matrix
R_range = [0.01 R_range];

i = 1;
for N = N_range
    j = 1;
    for R = R_range
        % Predition Model
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
        
        % Cost Function
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
        
        % Analytical Solution
        K_RHC = -[eye(m,m) zeros(m,(N-1)*m)]*inv(H)*F;
        
        % Stability Analysis
        stable(i,j) = max(abs(eig(A+B*K_RHC))) < 1;
        j = j+1;
    end;
    i = i+1;
end;

%% Plot
LineWidth = 1.5;
LineWidthAxes = 1;
FontName = 'Cambria';
FontSize = 16;

[N_grid R_grid] = meshgrid(N_range, R_range);
h = pcolor(N_grid,R_grid,double(stable)');
colormap([0.75 0 0; 0 0.7 0.3]);
set(gca,'LineWidth',LineWidthAxes);
set(h,'EdgeColor','none');
set(gca,'XTick',N_range);
set(gca,'YTick',[0:10:100]);
set(gca,'FontName',FontName,'FontSize',FontSize);
xlabel('Prediction Horizon {\it N}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');
ylabel('Input Weighting Factor {\it R}','FontName',FontName,'FontSize',FontSize,'Interpreter','tex');

