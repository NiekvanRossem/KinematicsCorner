%%-----------------------------------------------------------------------%%
% filename:         Solver.m
% author(s):        Niek van Rossem
% Creation date:    23-10-2024
%%-----------------------------------------------------------------------%%

%% Documentation
% -
%

%% Prepare workspace

% clear everything
%clear; close all; clc;

% set all figures to docked mode
set(0,'DefaultFigureWindowStyle','docked');

%% Input settings

% set wheel travel vertical speed
dTravel = 1;

% settings
Settings.RelTol = 1e-2;
Settings.AbsTol = 1e-2;
Settings.MaxStep = 1;
Settings.Axle = "Front";
Settings.Animation = "on";

%% DUT24 PUPs

% load suspension hardpoints
[Car, PUP] = DUT25_PUP(Settings);

% find loaded radius
RL = abs(PUP.r_WC_CH(3) - PUP.r_CP_CH(3));

% create state vector
r = vertcat( ...
    PUP.r_FUB, ...
    PUP.r_FLB, ...
    PUP.r_FUIF, ...
    PUP.r_FUIR, ...
    PUP.r_FLIF, ...
    PUP.r_FLIR, ...
    PUP.r_FOT, ...
    PUP.r_FIT, ...
    PUP.r_CP_CH, ...
    dTravel);

% set solver options
opts = odeset('RelTol', Settings.RelTol, 'AbsTol', Settings.AbsTol, 'MaxStep', Settings.MaxStep);

% solve for wheel travel sweep
[t, r_out] = ode45(@FiveLink, [0 200], r, opts);

VariableNames = [
    "r_FUB_x", "r_FUB_y", "r_FUB_z", ...
    "r_FLB_x", "r_FLB_y", "r_FLB_z", ...
    "r_FUIF_x", "r_FUIF_y", "r_FUIF_z", ...
    "r_FUIR_x", "r_FUIR_y", "r_FUIR_z", ...
    "r_FLIF_x", "r_FLIF_y", "r_FLIF_z", ...
    "r_FLIR_x", "r_FLIR_y", "r_FLIR_z", ...
    "r_FOT_x", "r_FOT_y", "r_FOT_z", ...
    "r_FIT_x", "r_FIT_y", "r_FIT_z", ...
    "r_CP_CH_x", "r_CP_CH_y", "r_CP_CH_z", ...
    "dTravel"];

% convert to table
output = array2table(r_out, "VariableNames", VariableNames);

%% extract generalised speeds

% allocate space
q = zeros(numel(t),6);

% loop over
for i = 1:numel(t)
    [~, q(i,:)] = FiveLink(t(i), transpose(r_out(i,:)));
end

% extract actual track width
Track = 2*output.r_CP_CH_y;

% extract wheel travel
Travel = output.r_CP_CH_z;

% calculate equivalent roll angle
Roll   = atan(Travel./(0.5*Track));

% calculate ride height
RideHeight = Car.RideHeight_0 - Travel;

output = addvars(output, q(:,1), q(:,2), 'After','r_CP_CH_z');
output = addvars(output, q(:,3), q(:,4), q(:,5), 'After','dTravel');

% generalised DoF
dWheelbase  = q(:,1);
dTrack      = q(:,2);
dCamber     = q(:,3);
dSpin       = q(:,4);
dToe        = q(:,5);
dTravel     = q(:,6);

% Suspension params
RollCentre  = Track./2.*(dTrack./dTravel);          % mm
CamberGainB = -dCamber./dTravel*180/pi;             % deg/mm
BumpSteer   = dToe./dTravel*180/pi;                 % deg/mm
CasterGain  = -dSpin./dTravel*180/pi;               % deg/mm
AntiRoll    = atan(dTrack./dTravel)*180/pi;         % deg
AntiPitch   = atan(-dWheelbase./dTravel)*180/pi;    % deg
LatGrad     = dTrack./dTravel;                      % mm/mm
LongGrad    = dWheelbase./dTravel;                  % mm/mm
CamberGainR = CamberGainB.*0.5.*Track.*sind(1)+1;   % deg/deg

% integrate to find camber and toe curves
Camber = cumtrapz(t, -dCamber);
Toe    = cumtrapz(t, dToe);

%% Plot output characteristics

% camber
figure("Name", "Camber characteristic");
subplot(1,2,1); hold all;
    title('Camber curve');
    plot(Camber, Travel, '.', 'MarkerSize', 2);
    xlabel('Camber angle (deg)');
    ylabel('Travel (mm)');
    xline(0, 'k-'); yline(0, 'k-');
    xlim([-0.1 0.1]); grid minor;
subplot(1,2,2); hold all;
    title('Camber gain');
    plot(CamberGainB, Travel, '.', 'MarkerSize', 2);
    xlabel('Camber gain (deg/mm)');
    ylabel('Travel (mm)');
    xline(0, 'k-'); yline(0, 'k-');
    xlim([-0.1 -0.05]);  grid minor;

% bump steer
figure("Name", "Bumpsteer characteristic");
subplot(1,2,1); hold all;
    title('Bumpsteer curve');
    plot(Toe, Travel, '.', 'MarkerSize', 2);
    xlabel('Toe angle (deg)');
    ylabel('Travel (mm)');
    xline(0, 'k-'); yline(0, 'k-');
    xlim([-0.02 0.02]); grid minor;
    annotation('textarrow', 0.5*[0.85 0.9],[0.4 0.4], 'String', 'toe in');
    annotation('textarrow', 0.5*[0.35 0.3],[0.4 0.4], 'String', 'toe out');
subplot(1,2,2); hold all;
    title('Toe gain');
    plot(BumpSteer, Travel, '.', 'MarkerSize', 2);
    xlabel('Toe gain (deg/mm)');
    ylabel('Travel (mm)');
    xline(0, 'k-'); yline(0, 'k-');
    xlim([-15e-3 15e-3]); grid minor;

figure("Name", "Roll centre height"); hold all; grid minor;
title('Roll centre curve');
plot(RideHeight, RollCentre, '.', 'MarkerSize', 2);
xlabel('Ride height (mm)');
ylabel('Roll centre height (mm)');

figure("Name", "Anti pitch angle");
subplot(1,2,1); hold all; grid minor;
    title('Lateral n-line angle curve');
    plot(RideHeight, AntiRoll, '.', 'MarkerSize', 2);
    xlabel('Ride height (mm)');
    ylabel('Anti-roll angle (deg)');
subplot(1,2,2); hold all; grid minor;
    title('Longitudinal n-line angle curve');
    plot(RideHeight, AntiPitch, '.', 'MarkerSize', 2);
    xlabel('Ride height (mm)');
    ylabel('Anti-pitch angle (deg)');

%% Plot animation
if Settings.Animation == "on"
    dynamicData = r_out(1,:);
    
    new_FUB = dynamicData(1:3);
    new_FLB = dynamicData(4:6);
    new_FOT = dynamicData(19:21);
    new_CP_CH = dynamicData(25:27);
    
    f = figure("Name", "Animation"); clf; view(90,0); hold all;
    box on; grid minor; axis equal;
    xlim([-150 150]); ylim([150 650]); zlim([-50 400]);
    
    h1 = plot3([new_FUB(1) PUP.r_FUIF(1)], [new_FUB(2) PUP.r_FUIF(2)], [new_FUB(3) PUP.r_FUIF(3)], 'o-', 'Color', 'red');
    h2 = plot3([new_FUB(1) PUP.r_FUIR(1)], [new_FUB(2) PUP.r_FUIR(2)], [new_FUB(3) PUP.r_FUIR(3)], 'o-', 'Color', 'red');
    h3 = plot3([new_FLB(1) PUP.r_FLIF(1)], [new_FLB(2) PUP.r_FLIF(2)], [new_FLB(3) PUP.r_FLIF(3)], 'o-', 'Color', 'red');
    h4 = plot3([new_FLB(1) PUP.r_FLIR(1)], [new_FLB(2) PUP.r_FLIR(2)], [new_FLB(3) PUP.r_FLIR(3)], 'o-', 'Color', 'red');
    h5 = plot3([new_FOT(1) PUP.r_FIT(1)], [new_FOT(2) PUP.r_FIT(2)], [new_FOT(3) PUP.r_FIT(3)], 'o-', 'Color', 'red');
    h6 = plot3(new_CP_CH(1), new_CP_CH(2), new_CP_CH(3), 'ro');
    
    for i = 1:length(r_out(:,1))
    
        dynamicData = r_out(i,:);
    
        new_FUB = dynamicData(1:3);
        new_FLB = dynamicData(4:6);
        new_FOT = dynamicData(19:21);
        new_CP_CH = dynamicData(25:27);
    
        % update data
        set(h1, 'XData', [new_FUB(1) PUP.r_FUIF(1)], 'YData', [new_FUB(2) PUP.r_FUIF(2)], 'ZData', [new_FUB(3) PUP.r_FUIF(3)]);
        set(h2, 'XData', [new_FUB(1) PUP.r_FUIR(1)], 'YData', [new_FUB(2) PUP.r_FUIR(2)], 'ZData', [new_FUB(3) PUP.r_FUIR(3)]);
        set(h3, 'XData', [new_FLB(1) PUP.r_FLIF(1)], 'YData', [new_FLB(2) PUP.r_FLIF(2)], 'ZData', [new_FLB(3) PUP.r_FLIF(3)]);
        set(h4, 'XData', [new_FLB(1) PUP.r_FLIR(1)], 'YData', [new_FLB(2) PUP.r_FLIR(2)], 'ZData', [new_FLB(3) PUP.r_FLIR(3)]);
        set(h5, 'XData', [new_FOT(1) PUP.r_FIT(1)], 'YData', [new_FOT(2) PUP.r_FIT(2)], 'ZData', [new_FOT(3) PUP.r_FIT(3)]);
        set(h6, 'XData', new_CP_CH(1), 'YData', new_CP_CH(2), 'ZData', new_CP_CH(3));

        % hold for some time for correct animation speed
        pause(Settings.MaxStep/1e2);
    end
end

%% Error test

% check distance error between lower and upper balljoints
errCalc.FUB = r_out(:,1:3);
errCalc.FLB = r_out(:,4:6);

dist = zeros(length(errCalc.FUB(:,1)),1);

% calculate distance between points for entire simulation
for n = 1:length(errCalc.FUB(:,1))
    dist(n) = sqrt( ...
        (errCalc.FUB(n,1) - errCalc.FLB(n,1))^2 + ...
        (errCalc.FUB(n,2) - errCalc.FLB(n,2))^2 + ...
        (errCalc.FUB(n,3) - errCalc.FLB(n,3))^2);
end

% find error
err = abs(dist - dist(1));

% plot result
figure("Name","Error check"); hold all; 
title({"Error growth rate check", "King pin length change"});
plot(t, err); grid minor;
xlabel('sim time (s)');
ylabel('error (mm)');

%% Constraints checking

% angular velocity
w_A = [dCamber'; dSpin'; dToe'];

% translational velocity
v_CP = [dWheelbase'; dTrack'; dTravel'];

r_P1_CP = r_out(:,1:3)' + PUP.r_FUIF - r_out(:,25:27)';
r_P2_CP = r_out(:,4:6)' + PUP.r_FUIR  - r_out(:,25:27)';
r_P3_CP = r_out(:,7:9)' + PUP.r_FLIF  - r_out(:,25:27)';
r_P4_CP = r_out(:,10:12)' + PUP.r_FLIR  - r_out(:,25:27)';
r_P5_CP = r_out(:,13:15)' + PUP.r_FIT  - r_out(:,25:27)';

test1 = zeros(numel(w_A(1,:)), 1);
test2 = zeros(numel(w_A(1,:)), 1);
test3 = zeros(numel(w_A(1,:)), 1);
test4 = zeros(numel(w_A(1,:)), 1);
test5 = zeros(numel(w_A(1,:)), 1);

% find veloctity vectors of each point
for n = 1%:numel(w_A(1,:))

    % velocity vectors [NOT CORRECT]
    v_P1 = v_CP(:,n) + cross(w_A(:,n), r_P1_CP(:,n));
    v_P2 = v_CP(:,n) + cross(w_A(:,n), r_P2_CP(:,n));
    v_P3 = v_CP(:,n) + cross(w_A(:,n), r_P3_CP(:,n));
    v_P4 = v_CP(:,n) + cross(w_A(:,n), r_P4_CP(:,n));
    v_P5 = v_CP(:,n) + cross(w_A(:,n), r_P5_CP(:,n));

    % link vectors
    r_P1_CH = r_out(n,1:3)' - PUP.r_FUIF;
    r_P2_CH = r_out(n,4:6)' - PUP.r_FUIR;
    r_P3_CH = r_out(n,7:9)' - PUP.r_FLIF;
    r_P4_CH = r_out(n,10:12)' - PUP.r_FLIR;
    r_P5_CH = r_out(n,13:15)' - PUP.r_FIT;

    % dot product (should equal zero)
    test1(n) = v_P1'*r_P1_CH;
    test2(n) = v_P2'*r_P2_CH;
    test3(n) = v_P3'*r_P3_CH;
    test4(n) = v_P4'*r_P4_CH;
    test5(n) = v_P5'*r_P5_CH;
end

disp(['max constraint offset (P1): ', num2str(round(max(abs(test1)), 3, "significant"))]);
disp(['max constraint offset (P2): ', num2str(round(max(abs(test2)), 3, "significant"))]);
disp(['max constraint offset (P3): ', num2str(round(max(abs(test3)), 3, "significant"))]);
disp(['max constraint offset (P4): ', num2str(round(max(abs(test4)), 3, "significant"))]);
disp(['max constraint offset (P5): ', num2str(round(max(abs(test5)), 3, "significant"))]);