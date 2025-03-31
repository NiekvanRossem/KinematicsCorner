clc; close all; clear;

dTravel = 1;

%% PUPs from Suh's paper

% contact patch
r_CP_CH    = [ -54.137;     718.938;   -319.000];

Track = r_CP_CH(2);

% wheel centre
r_WC_CH    = [ -54.137;     718.938;    -45.000];

% suspension PUPs
r_FUB   = [ -49.496;     603.805;     71.853];
r_FLB   = [ -58.341;     624.368;   -150.141];
r_FUIF  = [-127.000;     383.250;    106.200];
r_FUIR  = [  32.000;     383.250;     95.200];
r_FLIF  = [-358.000;     314.750;    -67.600];
r_FLIR  = [ -55.650;     232.200;   -110.650];
r_FIT   = [-158.000;     360.500;    -66.600];
r_FOT   = [-174.166;     633.511;    -98.290];

%%

RL = abs(r_WC_CH(3)-r_CP_CH(3));

% 1st link (FUF)
r_P1_CH = r_FUB - r_FUIF;
r_P1_CP = r_FUB - r_CP_CH;

% 2nd link (FUR)
r_P2_CH = r_FUB - r_FUIR;
r_P2_CP = r_FUB - r_CP_CH;

% 3rd link (FLF)
r_P3_CH = r_FLB - r_FLIF;
r_P3_CP = r_FLB - r_CP_CH;

% 4th link (FLR)
r_P4_CH = r_FLB - r_FLIR;
r_P4_CP = r_FLB - r_CP_CH;

% 5th link (TIE)
r_P5_CH = r_FOT - r_FIT;
r_P5_CP = r_FOT - r_CP_CH;

%% Convert to matrix form and solve

Q1 = [r_P1_CP(1); r_P2_CP(1); r_P3_CP(1); r_P4_CP(1); r_P5_CP(1)];
Q2 = [r_P1_CP(2); r_P2_CP(2); r_P3_CP(2); r_P4_CP(2); r_P5_CP(2)];
Q3 = [r_P1_CP(3); r_P2_CP(3); r_P3_CP(3); r_P4_CP(3); r_P5_CP(3)];
Q4 = [r_P1_CH(1); r_P2_CH(1); r_P3_CH(1); r_P4_CH(1); r_P5_CH(1)];
Q5 = [r_P1_CH(2); r_P2_CH(2); r_P3_CH(2); r_P4_CH(2); r_P5_CH(2)];
Q6 = [r_P1_CH(3); r_P2_CH(3); r_P3_CH(3); r_P4_CH(3); r_P5_CH(3)];

A = [Q4, Q5, Q2.*Q6-Q3.*Q5, Q3.*Q4-Q1.*Q6, Q1.*Q5-Q2.*Q4];
b = -Q6;

q = A\b;

%% Characteristics

% generalised DoF
dWheelbase  = q(1);
dTrack      = q(2);
dCamber     = q(3);
dSpin       = q(4);
dToe        = q(5);

% Suspension params
RollCentre  = Track/2*dTrack;           % mm
CamberGain1 = -dCamber*180/pi;          % deg/mm
BumpSteer   = dToe*180/pi;              % deg/mm
CasterGain  = -dSpin*180/pi;            % deg/mm
AntiRoll    = atan(dTrack)*180/pi;      % deg
AntiPitch   = atan(-dWheelbase)*180/pi; % deg
LatGrad     = dTrack/dTravel;           % mm/mm
LongGrad    = dWheelbase/dTravel;       % mm/mm

% cambergain in roll
CamberGain2 = CamberGain1*0.5*Track*sind(1)+1;

%% Print results
disp('~~ RESULTS ~~');
disp(['Anti roll angle:       ', num2str(round(AntiRoll,2)), ' deg']);
disp(['Anti pitch angle:      ', num2str(round(AntiPitch,2)), ' deg']);
disp(['Roll centre height:    ', num2str(round(RollCentre,1)), ' mm']);
disp(['Bump steer:            ', num2str(round(BumpSteer,3)), ' deg/mm']);
disp(['Camber gain (bump):    ', num2str(round(CamberGain1,3)), ' deg/mm']);
disp(['Camber gain (roll):    ', num2str(round(CamberGain2,3)), ' deg/deg']);
disp(['Caster gain:           ', num2str(round(CasterGain,3)), ' deg/mm']);
disp(['Lateral gradient:      ', num2str(round(LatGrad,3)), ' mm/mm']);
disp(['Longitudinal gradient: ', num2str(round(LatGrad,3)), ' mm/mm']);

%% Calculate velocity vectors

w_A = [dCamber; dSpin; dToe];
v_CP = [dWheelbase; dTrack; 1];

v_P1 = v_CP + cross(w_A, r_P1_CP);
v_P2 = v_CP + cross(w_A, r_P2_CP);
v_P3 = v_CP + cross(w_A, r_P3_CP);
v_P4 = v_CP + cross(w_A, r_P4_CP);
v_P5 = v_CP + cross(w_A, r_P5_CP);

%% Plot geometry
figure; hold all; view(45,30);
box on; grid minor; axis equal;
plot3([r_FUB(1) r_FUIF(1)], [r_FUB(2) r_FUIF(2)], [r_FUB(3) r_FUIF(3)], 'o-', 'Color', 'red');
plot3([r_FUB(1) r_FUIR(1)], [r_FUB(2) r_FUIR(2)], [r_FUB(3) r_FUIR(3)], 'o-', 'Color', 'red');
plot3([r_FLB(1) r_FLIF(1)], [r_FLB(2) r_FLIF(2)], [r_FLB(3) r_FLIF(3)], 'o-', 'Color', 'red');
plot3([r_FLB(1) r_FLIR(1)], [r_FLB(2) r_FLIR(2)], [r_FLB(3) r_FLIR(3)], 'o-', 'Color', 'red');
plot3([r_FOT(1) r_FIT(1)], [r_FOT(2) r_FIT(2)], [r_FOT(3) r_FIT(3)], 'o-', 'Color', 'red');
plot3(r_CP_CH(1), r_CP_CH(2), r_CP_CH(3), 'ro');

%% Plot tyre

[X, Z, Y] = cylinder(RL, 16);

w = 7.5*25.4;

surf(X+r_WC_CH(1), Y*w+r_WC_CH(2)-w/2, Z+r_WC_CH(3), 'FaceColor', 'none', 'EdgeColor', 'k');

%% Plot velocity arrows

quiver3(r_FUB(1), r_FUB(2), r_FUB(3), 100*v_P1(1), 100*v_P1(2), 100*v_P1(3), 'Color', 'blue');
quiver3(r_FLB(1), r_FLB(2), r_FLB(3), 100*v_P3(1), 100*v_P3(2), 100*v_P3(3), 'Color', 'blue');
quiver3(r_FOT(1), r_FOT(2), r_FOT(3), 100*v_P5(1), 100*v_P5(2), 100*v_P5(3), 'Color', 'blue');
quiver3(r_CP_CH(1), r_CP_CH(2), r_CP_CH(3), 100*v_CP(1), 100*v_CP(2), 100*v_CP(3), 'Color', 'blue');

%% Constraint check

disp(['err (P1): ',num2str(round(v_P1'*r_P1_CH, 3, "significant"))]);
disp(['err (P2): ',num2str(round(v_P2'*r_P2_CH, 3, "significant"))]);
disp(['err (P3): ',num2str(round(v_P3'*r_P3_CH, 3, "significant"))]);
disp(['err (P4): ',num2str(round(v_P4'*r_P4_CH, 3, "significant"))]);
disp(['err (P5): ',num2str(round(v_P5'*r_P5_CH, 3, "significant"))]);
