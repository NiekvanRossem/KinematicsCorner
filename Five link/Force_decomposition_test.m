%% Prepare workspace

% clear everything
clear; close all; clc;

% set all figures to docked mode
set(0,'DefaultFigureWindowStyle','docked');
Settings.Axle = "Front";

%% Load suspension hardpoints

% load suspension hardpoints
[Car, PUP] = Suh_PUP(Settings);

% find loaded radius
Car.RL = abs(PUP.r_WC_O(3) - PUP.r_CP_O(3));

% Force and moment vectors
F_CP = [   0.000;     0.000;     1.000];   % N
M_CP = [   0.000;     0.000;     0.000];   % Nm

%%

% 1st link (FUF)
r_P1_CH = PUP.r_P1o - PUP.r_P1i;    % from chassis
r_P1_CP = PUP.r_P1o - PUP.r_CP_O;   % from wheel centre

% 2nd link (FUR)
r_P2_CH = PUP.r_P2o - PUP.r_P2i;
r_P2_CP = PUP.r_P2o - PUP.r_CP_O;

% 3rd link (FLF)
r_P3_CH = PUP.r_P3o - PUP.r_P3i;
r_P3_CP = PUP.r_P3o - PUP.r_CP_O;

% 4th link (FLR)
r_P4_CH = PUP.r_P4o - PUP.r_P4i;
r_P4_CP = PUP.r_P4o - PUP.r_CP_O;

% 5th link (TIE)
r_P5_CH = PUP.r_P5o - PUP.r_P5i;
r_P5_CP = PUP.r_P5o - PUP.r_CP_O;

% 5th link (SDS)
r_P6_CH = PUP.r_P6o - PUP.r_P6i;
r_P6_CP = PUP.r_P6o - PUP.r_CP_O;

% Force equilibrium part of matrix

u_P1_CH = r_P1_CH/norm(r_P1_CH);
u_P2_CH = r_P2_CH/norm(r_P2_CH);
u_P3_CH = r_P3_CH/norm(r_P3_CH);
u_P4_CH = r_P4_CH/norm(r_P4_CH);
u_P5_CH = r_P5_CH/norm(r_P5_CH);
u_P6_CH = r_P6_CH/norm(r_P6_CH);

A1 = [u_P1_CH, u_P2_CH, u_P3_CH, u_P4_CH, u_P5_CH, u_P6_CH];

% Moment equilibrium part of matrix

A21 = cross(r_P1_CP, u_P1_CH);
A22 = cross(r_P2_CP, u_P2_CH);
A23 = cross(r_P3_CP, u_P3_CH);
A24 = cross(r_P4_CP, u_P4_CH);
A25 = cross(r_P5_CP, u_P5_CH);
A26 = cross(r_P6_CP, u_P6_CH);

A2 = [A21, A22, A23, A24, A25, A26];

%% Final equation and solution

A = [A1; A2];
b = -[F_CP; 1e3*M_CP];

x = A\b;

%% Display results

disp('=== Summary ===');
disp(['F1 = ', num2str(round(x(1), 4, 'Significant')), ' N (FUF)']);
disp(['F2 = ', num2str(round(x(2), 4, 'Significant')), ' N (FUR)']);
disp(['F3 = ', num2str(round(x(3), 4, 'Significant')), ' N (FLF)']);
disp(['F4 = ', num2str(round(x(4), 4, 'Significant')), ' N (FLR)']);
disp(['F5 = ', num2str(round(x(5), 4, 'Significant')), ' N (TIE)']);
disp(['F6 = ', num2str(round(x(6), 4, 'Significant')), ' N (SDS)']);
disp('');
disp('===  Input  ===');
disp(['Fx = ', num2str(round(F_CP(1), 4, 'Significant')), ' N']);
disp(['Fy = ', num2str(round(F_CP(2), 4, 'Significant')), ' N']);
disp(['Fz = ', num2str(round(F_CP(3), 4, 'Significant')), ' N']);
disp(['Mx = ', num2str(round(M_CP(1), 4, 'Significant')), ' Nm']);
disp(['My = ', num2str(round(M_CP(2), 4, 'Significant')), ' Nm']);
disp(['Mz = ', num2str(round(M_CP(3), 4, 'Significant')), ' Nm']);
