%%-----------------------------------------------------------------------%%
% filename:         Solver.m
% author(s):        Niek van Rossem
% Creation date:    23-10-2024
% Documentation
% -
%%-----------------------------------------------------------------------%%

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
Car.RL = abs(PUP.r_WC_O(3) - PUP.r_CP_O(3));

% create state vector
r = vertcat( ...
    PUP.r_P1o, ...
    PUP.r_P2o, ...
    PUP.r_P3o, ...
    PUP.r_P4o, ...
    PUP.r_P5o, ...
    PUP.r_P1i, ...
    PUP.r_P2i, ...
    PUP.r_P3i, ...
    PUP.r_P4i, ...
    PUP.r_P5i, ...
    PUP.r_CP_O, ...
    dTravel);

% set solver options
opts = odeset('RelTol', Settings.RelTol, 'AbsTol', Settings.AbsTol, 'MaxStep', Settings.MaxStep);

% solve for wheel travel sweep
[t, r_out] = ode45(@FiveLink, [0 200], r, opts);

Settings.VariableNames = [
    "t", ...
    "r_P1o_x", "r_P1o_y", "r_P1o_z", ...
    "r_P2o_x", "r_P2o_y", "r_P2o_z", ...
    "r_P3o_x", "r_P3o_y", "r_P3o_z", ...
    "r_P4o_x", "r_P4o_y", "r_P4o_z", ...
    "r_P5o_x", "r_P5o_y", "r_P5o_z", ...
    "r_P1i_x", "r_P1i_y", "r_P1i_z", ...
    "r_P2i_x", "r_P2i_y", "r_P2i_z", ...
    "r_P3i_x", "r_P3i_y", "r_P3i_z", ...
    "r_P4i_x", "r_P4i_y", "r_P4i_z", ...
    "r_P5i_x", "r_P5i_y", "r_P5i_z", ...
    "r_CP_O_x", "r_CP_O_y", "r_CP_O_z", ...
    "dTravel"];

% convert to table
output = array2table([t, r_out], "VariableNames", Settings.VariableNames);
output.dTravel = [];

%% extract generalised speeds

% allocate space for generalised speeds
q_out = zeros(numel(output.t),6);

% loop over all time steps
for i = 1:numel(output.t)
    [~, q_out(i,:)] = FiveLink(output.t(i), transpose(r_out(i,:)));
end

% generalised DoF
output.dWheelbase  = q_out(:,1);
output.dTrack      = q_out(:,2);
output.dTravel     = q_out(:,6);
output.dCamber     = q_out(:,3);
output.dSpin       = q_out(:,4);
output.dToe        = q_out(:,5);

clear i; clear r_out; clear q_out; clear t; clear dTravel;

% extract actual track width
output.Track = 2*output.r_CP_O_y;

% extract wheel travel
output.Travel = output.r_CP_O_z;

% calculate equivalent roll angle
output.Roll   = 180/pi*atan(output.Travel./(0.5*output.Track));

% calculate ride height
output.RideHeight = Car.RH - output.Travel;

% Suspension params
output.RollCentre  = output.Track./2.*(output.dTrack./output.dTravel);          % mm
output.CamberGainB = -output.dCamber./output.dTravel*180/pi;             % deg/mm
output.BumpSteer   = output.dToe./output.dTravel*180/pi;                 % deg/mm
output.CasterGain  = -output.dSpin./output.dTravel*180/pi;               % deg/mm
output.AntiRoll    = atan(output.dTrack./output.dTravel)*180/pi;         % deg
output.AntiPitch   = atan(-output.dWheelbase./output.dTravel)*180/pi;    % deg
output.LatGrad     = output.dTrack./output.dTravel;                      % mm/mm
output.LongGrad    = output.dWheelbase./output.dTravel;                  % mm/mm
output.CamberGainR = output.CamberGainB.*0.5.*output.Track.*sind(1)+1;   % deg/deg

% integrate to find camber and toe curves
output.Camber = cumtrapz(output.t, -output.dCamber*180/pi);
output.Toe    = cumtrapz(output.t, output.dToe*180/pi);

%% Plot output characteristics

% camber
figure(1);%"Name", "Camber characteristic");
subplot(1,2,1); hold all;
    title('Camber curve');
    plot(output.Camber, output.Travel, '.', 'MarkerSize', 2);
    xlabel('Camber angle (deg)');
    ylabel('Travel (mm)');
    xline(0, 'k-'); yline(0, 'k-');
    xlim([-5 5]); grid minor;
subplot(1,2,2); hold all;
    title('Camber gain');
    plot(output.CamberGainB, output.Travel, '.', 'MarkerSize', 2);
    xlabel('Camber gain (deg/mm)');
    ylabel('Travel (mm)');
    xline(0, 'k-'); yline(0, 'k-');
    xlim([-0.1 -0.05]);  grid minor;

% bump steer
figure(2);%"Name", "Bumpsteer characteristic");
subplot(1,2,1); hold all;
    title('Bumpsteer curve');
    plot(output.Toe, output.Travel, '.', 'MarkerSize', 2);
    xlabel('Toe angle (deg)');
    ylabel('Travel (mm)');
    xline(0, 'k-'); yline(0, 'k-');
    xlim([-0.5 0.5]); grid minor;
    annotation('textarrow', 0.5*[0.85 0.9],[0.4 0.4], 'String', 'toe in');
    annotation('textarrow', 0.5*[0.35 0.3],[0.4 0.4], 'String', 'toe out');
subplot(1,2,2); hold all;
    title('Toe gain');
    plot(output.BumpSteer, output.Travel, '.', 'MarkerSize', 2);
    xlabel('Toe gain (deg/mm)');
    ylabel('Travel (mm)');
    xline(0, 'k-'); yline(0, 'k-');
    xlim([-15e-3 15e-3]); grid minor;

figure(3);%"Name", "Roll centre height"); hold all; grid minor;
title('Roll centre curve');
plot(output.RideHeight, output.RollCentre, '.', 'MarkerSize', 2);
xlabel('Ride height (mm)');
ylabel('Roll centre height (mm)');

figure(4);%"Name", "Anti pitch angle");
subplot(1,2,1); hold all; grid minor;
    title('Lateral n-line angle curve');
    plot(output.RideHeight, output.AntiRoll, '.', 'MarkerSize', 2);
    xlabel('Ride height (mm)');
    ylabel('Anti-roll angle (deg)');
subplot(1,2,2); hold all; grid minor;
    title('Longitudinal n-line angle curve');
    plot(output.RideHeight, output.AntiPitch, '.', 'MarkerSize', 2);
    xlabel('Ride height (mm)');
    ylabel('Anti-pitch angle (deg)');

%% Plot animation
if Settings.Animation == "on"
    dynamicData = r_out(1,:);
    
    new_FUB = [output.r_P1o_x, output.r_P1o_y, output.r_P1o_z];
    new_FLB = [output.r_P3o_x, output.r_P3o_y, output.r_P3o_z];
    new_FOT = [output.r_P5o_x, output.r_P5o_y, output.r_P5o_z];
    new_CP_O = [output.r_CP_O_x, output.r_CP_O_y, output.r_CP_O_z];
    
    f = figure("Name", "Animation"); clf; view(90,0); hold all;
    box on; grid minor; axis equal;
    xlim([-150 150]); ylim([150 650]); zlim([-50 400]);
    
    h1 = plot3([new_FUB(1) PUP.r_FUIF(1)], [new_FUB(2) PUP.r_FUIF(2)], [new_FUB(3) PUP.r_FUIF(3)], 'o-', 'Color', 'red');
    h2 = plot3([new_FUB(1) PUP.r_FUIR(1)], [new_FUB(2) PUP.r_FUIR(2)], [new_FUB(3) PUP.r_FUIR(3)], 'o-', 'Color', 'red');
    h3 = plot3([new_FLB(1) PUP.r_FLIF(1)], [new_FLB(2) PUP.r_FLIF(2)], [new_FLB(3) PUP.r_FLIF(3)], 'o-', 'Color', 'red');
    h4 = plot3([new_FLB(1) PUP.r_FLIR(1)], [new_FLB(2) PUP.r_FLIR(2)], [new_FLB(3) PUP.r_FLIR(3)], 'o-', 'Color', 'red');
    h5 = plot3([new_FOT(1) PUP.r_FIT(1)], [new_FOT(2) PUP.r_FIT(2)], [new_FOT(3) PUP.r_FIT(3)], 'o-', 'Color', 'red');
    h6 = plot3(new_CP_O(1), new_CP_O(2), new_CP_O(3), 'ro');
    
    for i = 1:length(r_out(:,1))
    
        dynamicData = r_out(i,:);
    
        new_FUB = dynamicData(1:3);
        new_FLB = dynamicData(4:6);
        new_FOT = dynamicData(19:21);
        new_CP_O = dynamicData(25:27);
    
        % update data
        set(h1, 'XData', [new_FUB(1) PUP.r_P1i(1)], 'YData', [new_FUB(2) PUP.r_FUIF(2)], 'ZData', [new_FUB(3) PUP.r_FUIF(3)]);
        set(h2, 'XData', [new_FUB(1) PUP.r_P2i(1)], 'YData', [new_FUB(2) PUP.r_FUIR(2)], 'ZData', [new_FUB(3) PUP.r_FUIR(3)]);
        set(h3, 'XData', [new_FLB(1) PUP.r_P3i(1)], 'YData', [new_FLB(2) PUP.r_FLIF(2)], 'ZData', [new_FLB(3) PUP.r_FLIF(3)]);
        set(h4, 'XData', [new_FLB(1) PUP.r_P4i(1)], 'YData', [new_FLB(2) PUP.r_FLIR(2)], 'ZData', [new_FLB(3) PUP.r_FLIR(3)]);
        set(h5, 'XData', [new_FOT(1) PUP.r_P5i(1)], 'YData', [new_FOT(2) PUP.r_FIT(2)], 'ZData', [new_FOT(3) PUP.r_FIT(3)]);
        set(h6, 'XData', new_CP_O(1), 'YData', new_CP_O(2), 'ZData', new_CP_O(3));

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