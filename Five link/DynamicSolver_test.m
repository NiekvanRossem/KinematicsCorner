%%-----------------------------------------------------------------------%%
% filename:         DynamicSolver_test.m
% author(s):        Niek van Rossem
% Creation date:    23-10-2024
% Documentation:
%       Test script for the five link solver.
%%-----------------------------------------------------------------------%%

%% Prepare workspace

% clear everything
clear; close all; clc;

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
Settings.Animation = "off";

%% Load suspension hardpoints

% load suspension hardpoints
[Car, PUP] = Suh_PUP(Settings);

% find loaded radius
Car.RL = abs(PUP.r_WC_O(3) - PUP.r_CP_O(3));

%% Solve

% create state vector
r = vertcat( ...
    PUP.r_P1o, ...
    PUP.r_P2o, ...
    PUP.r_P3o, ...
    PUP.r_P4o, ...
    PUP.r_P5o, ...
    PUP.r_P6o, ...
    PUP.r_P1i, ...
    PUP.r_P2i, ...
    PUP.r_P3i, ...
    PUP.r_P4i, ...
    PUP.r_P5i, ...
    PUP.r_P6i, ...
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
    "r_P6o_x", "r_P6o_y", "r_P6o_z", ...
    "r_P1i_x", "r_P1i_y", "r_P1i_z", ...
    "r_P2i_x", "r_P2i_y", "r_P2i_z", ...
    "r_P3i_x", "r_P3i_y", "r_P3i_z", ...
    "r_P4i_x", "r_P4i_y", "r_P4i_z", ...
    "r_P5i_x", "r_P5i_y", "r_P5i_z", ...
    "r_P6i_x", "r_P6i_y", "r_P6i_z", ...
    "r_CP_O_x", "r_CP_O_y", "r_CP_O_z", ...
    "dTravel"];

% convert to table
output = array2table([t, r_out], "VariableNames", Settings.VariableNames);
output.dTravel = [];

%% extract generalised speeds

% allocate space for generalised speeds
q_out = zeros(numel(output.t),6);

% loop over all time steps
for n = 1:numel(output.t)
    [~, q_out(n,:)] = FiveLink(output.t(n), transpose(r_out(n,:)));
end

% generalised DoF
output.dWheelbase  = q_out(:,1);
output.dTrack      = q_out(:,2);
output.dTravel     = q_out(:,6);
output.dCamber     = q_out(:,3);
output.dSpin       = q_out(:,4);
output.dToe        = q_out(:,5);

clear r_out; clear q_out; clear t; clear dTravel;

% extract actual track width
output.Track = 2*output.r_CP_O_y;

% extract wheel travel
output.Travel = output.r_CP_O_z;

% calculate equivalent roll angle and roll rate
output.Roll   = 180/pi*atan(output.Travel./(0.5*output.Track));
output.dRoll  = 180/pi*atan(output.dTravel./(0.5*output.Track));

% calculate ride height
output.RideHeight = Car.RH - output.Travel;

% Suspension params
output.RollCentre   = output.Track./2.*(output.dTrack./output.dTravel);     % mm
output.CamberGainB  = -output.dCamber./output.dTravel*180/pi;               % deg/mm
output.BumpSteer    = output.dToe./output.dTravel*180/pi;                   % deg/mm
output.CasterGain   = -output.dSpin./output.dTravel*180/pi;                 % deg/mm
output.AntiRoll     = atan(output.dTrack./output.dTravel)*180/pi;           % deg
output.AntiPitch    = atan(-output.dWheelbase./output.dTravel)*180/pi;      % deg
output.LatGrad      = output.dTrack./output.dTravel;                        % mm/mm
output.LongGrad     = output.dWheelbase./output.dTravel;                    % mm/mm
output.CamberGainR  = output.CamberGainB.*0.5.*output.Track.*sind(1)+1;     % deg/deg
output.RollSteer    = 180/pi*output.dToe./output.dRoll;                      % deg/deg

% integrate to find camber and toe curves
output.Camber = cumtrapz(output.t, -output.dCamber*180/pi);
output.Toe    = cumtrapz(output.t, output.dToe*180/pi);

% damper length
temp = sqrt((output.r_P6o_x - output.r_P6i_x).^2 ...
    + (output.r_P6o_y - output.r_P6i_y).^2 ...
    + (output.r_P6o_z - output.r_P6i_z).^2);

% damper travel
output.DamperTravel = temp(1) - temp;
clear temp;

% displacement based motion ratio
for n = 1:length(output.t)
    if n == 1
        output.MR1(n) = nan;
    else
        output.MR1(n) = (output.DamperTravel(n) -output.DamperTravel(n-1)) ./ (output.Travel(n) - output.Travel(n-1));
    end
    if abs(output.MR1(n)) > 10 || abs(output.MR1(n)) < 0.05
        output.MR1(n) = nan;
    end
end

%% Plot output characteristics

% camber
figure(1);
subplot(1,2,1); hold all; box on; grid minor;
    title({'Camber curve', 'With respect to the chassis'});
    plot(output.Camber, output.Travel, '.', 'MarkerSize', 2);
    xlabel('Camber angle (deg)');
    ylabel('Travel (mm)');
    xline(0, 'k-'); yline(0, 'k-');
    xlim([-5 5]);
subplot(1,2,2); hold all; box on; grid minor;
    title('Camber gain in bump');
    plot(output.CamberGainB, output.Travel, '.', 'MarkerSize', 2);
    xlabel('Camber gain (deg/mm)');
    ylabel('Travel (mm)');
    xline(0, 'k-');
    
% bump steer
figure(2);
subplot(1,2,1); hold all; box on; grid minor;
    title('Bumpsteer curve');
    plot(output.Toe, output.Travel, '.', 'MarkerSize', 2);
    xlabel('Toe angle (deg)');
    ylabel('Travel (mm)');
    xline(0, 'k-'); yline(0, 'k-');
    xlim([-0.5 0.5]);
    annotation('textarrow', 0.5*[0.85 0.9],[0.4 0.4], 'String', 'toe in');
    annotation('textarrow', 0.5*[0.35 0.3],[0.4 0.4], 'String', 'toe out');
subplot(1,2,2); hold all; box on; grid minor;
    title('Toe gain');
    plot(output.BumpSteer, output.Travel, '.', 'MarkerSize', 2);
    xlabel('Toe gain (deg/mm)');
    ylabel('Travel (mm)');
    xline(0, 'k-'); yline(0, 'k-');
    xlim([-15e-3 15e-3]);

% roll centre height
figure(3);
title('Roll centre curve');
hold all; box on; grid minor;
plot(output.RideHeight, output.RollCentre, '.', 'MarkerSize', 2);
xline(Car.RH, 'k-'); yline(0, 'k-');
xlabel('Ride height (mm)');
ylabel('Roll centre height (mm)');

% n-line angles
figure(4);
subplot(1,2,1); hold all; box on; grid minor;
    title({'Lateral n-line angle curve', 'With respect to the chassis'});
    plot(output.RideHeight, output.AntiRoll, '.', 'MarkerSize', 2);
    xline(Car.RH, 'k-'); yline(0, 'k-');
    xlabel('Ride height (mm)');
    ylabel('Anti-roll angle (deg)');
subplot(1,2,2); hold all; box on; grid minor;
    title({'Longitudinal n-line angle curve', 'With respect to the chassis'});
    plot(output.RideHeight, output.AntiPitch, '.', 'MarkerSize', 2);
    xline(Car.RH, 'k-'); yline(0, 'k-');
    xlabel('Ride height (mm)');
    ylabel('Anti-pitch angle (deg)');

% camber in roll
figure(5); 
subplot(1,2,1); hold all; box on; grid minor;
    title({'Camber angle versus roll angle', 'With respect to the ground plane'});
    plot(output.Roll, output.Camber+output.Roll);
    xline(0, 'k-'); yline(0, 'k-');
    ylabel("Camber (deg)");
    xlabel("Roll angle (deg)");
subplot(1,2,2); hold all; box on; grid minor;
    title({'Camber gain in roll'});
    plot(output.Roll, output.CamberGainR); 
    xline(0, 'k-');
    ylabel("Camber gain in roll (deg/deg)");
    xlabel("Roll angle (deg)");

figure(6); 
subplot(1,2,1); hold all; box on; grid minor;
    title('Steer versus roll angle');
    plot(output.Roll, output.Toe);
    xlabel('Roll angle (deg)');
    ylabel('Steer angle (deg)');
    xline(0, 'k-'); yline(0, 'k-');
subplot(1,2,2); hold all; box on; grid minor;
    title('Roll steer coefficient over roll angle');
    plot(output.Roll, output.RollSteer);
    xlabel('Roll angle (deg)');
    ylabel('Roll steer (deg/deg)');
    xline(0, 'k-'); yline(0, 'k-');

%% Plot animation

if Settings.Animation == "on"
    
    % initialise outer PUP locations
    Animation.new_P1o = [output.r_P1o_x(1), output.r_P1o_y(1), output.r_P1o_z(1)];
    Animation.new_P2o = [output.r_P2o_x(1), output.r_P2o_y(1), output.r_P2o_z(1)];
    Animation.new_P3o = [output.r_P3o_x(1), output.r_P3o_y(1), output.r_P3o_z(1)];
    Animation.new_P4o = [output.r_P4o_x(1), output.r_P4o_y(1), output.r_P4o_z(1)];
    Animation.new_P5o = [output.r_P5o_x(1), output.r_P5o_y(1), output.r_P5o_z(1)];
    Animation.new_P6o = [output.r_P6o_x(1), output.r_P6o_y(1), output.r_P6o_z(1)];
    Animation.new_CP_O = [output.r_CP_O_x(1), output.r_CP_O_y(1), output.r_CP_O_z(1)];
    
    % initialise figure
    Animation.f = figure("Name", "Animation"); clf; view(135,30); hold all;
    box on; grid minor; axis equal;
    %xlim([-150 150]); ylim([150 650]); zlim([-50 400]);
    xlabel("X (mm)"); ylabel("Y (mm)"); zlabel("Z (mm)"); 

    % plot links
    Animation.h1 = plot3([Animation.new_P1o(1) PUP.r_P1i(1)], [Animation.new_P1o(2) PUP.r_P1i(2)], [Animation.new_P1o(3) PUP.r_P1i(3)], 'o-', 'Color', 'red');
    Animation.h2 = plot3([Animation.new_P2o(1) PUP.r_P2i(1)], [Animation.new_P2o(2) PUP.r_P2i(2)], [Animation.new_P2o(3) PUP.r_P2i(3)], 'o-', 'Color', 'red');
    Animation.h3 = plot3([Animation.new_P3o(1) PUP.r_P3i(1)], [Animation.new_P3o(2) PUP.r_P3i(2)], [Animation.new_P3o(3) PUP.r_P3i(3)], 'o-', 'Color', 'red');
    Animation.h4 = plot3([Animation.new_P4o(1) PUP.r_P4i(1)], [Animation.new_P4o(2) PUP.r_P4i(2)], [Animation.new_P4o(3) PUP.r_P4i(3)], 'o-', 'Color', 'red');
    Animation.h5 = plot3([Animation.new_P5o(1) PUP.r_P5i(1)], [Animation.new_P5o(2) PUP.r_P5i(2)], [Animation.new_P5o(3) PUP.r_P5i(3)], 'o-', 'Color', 'red');
    Animation.h6 = plot3([Animation.new_P6o(1) PUP.r_P6i(1)], [Animation.new_P6o(2) PUP.r_P6i(2)], [Animation.new_P6o(3) PUP.r_P6i(3)], 'o-', 'Color', 'red');

    % plot tyre contact patch
    Animation.h7 = plot3(Animation.new_CP_O(1), Animation.new_CP_O(2), Animation.new_CP_O(3), 'ro');
    
    % update data in figure
    for n = 1:length(output.t)
    
        % find new PUP coordinates
        Animation.new_P1o = [output.r_P1o_x(n), output.r_P1o_y(n), output.r_P1o_z(n)];
        Animation.new_P2o = [output.r_P2o_x(n), output.r_P2o_y(n), output.r_P2o_z(n)];
        Animation.new_P3o = [output.r_P3o_x(n), output.r_P3o_y(n), output.r_P3o_z(n)];
        Animation.new_P4o = [output.r_P4o_x(n), output.r_P4o_y(n), output.r_P4o_z(n)];
        Animation.new_P5o = [output.r_P5o_x(n), output.r_P5o_y(n), output.r_P5o_z(n)];
        Animation.new_P6o = [output.r_P6o_x(n), output.r_P6o_y(n), output.r_P6o_z(n)];
        Animation.new_CP_O = [output.r_CP_O_x(n), output.r_CP_O_y(n), output.r_CP_O_z(n)];
    
        % update data
        set(Animation.h1, 'XData', [Animation.new_P1o(1) PUP.r_P1i(1)], 'YData', [Animation.new_P1o(2) PUP.r_P1i(2)], 'ZData', [Animation.new_P1o(3) PUP.r_P1i(3)]);
        set(Animation.h2, 'XData', [Animation.new_P2o(1) PUP.r_P2i(1)], 'YData', [Animation.new_P2o(2) PUP.r_P2i(2)], 'ZData', [Animation.new_P2o(3) PUP.r_P2i(3)]);
        set(Animation.h3, 'XData', [Animation.new_P3o(1) PUP.r_P3i(1)], 'YData', [Animation.new_P3o(2) PUP.r_P3i(2)], 'ZData', [Animation.new_P3o(3) PUP.r_P3i(3)]);
        set(Animation.h4, 'XData', [Animation.new_P4o(1) PUP.r_P4i(1)], 'YData', [Animation.new_P4o(2) PUP.r_P4i(2)], 'ZData', [Animation.new_P4o(3) PUP.r_P4i(3)]);
        set(Animation.h5, 'XData', [Animation.new_P5o(1) PUP.r_P5i(1)], 'YData', [Animation.new_P5o(2) PUP.r_P5i(2)], 'ZData', [Animation.new_P5o(3) PUP.r_P5i(3)]);
        set(Animation.h6, 'XData', [Animation.new_P6o(1) PUP.r_P6i(1)], 'YData', [Animation.new_P6o(2) PUP.r_P6i(2)], 'ZData', [Animation.new_P6o(3) PUP.r_P6i(3)]);
        set(Animation.h7, 'XData', Animation.new_CP_O(1), 'YData', Animation.new_CP_O(2), 'ZData', Animation.new_CP_O(3));

        % hold for some time for correct animation speed
        pause(Settings.MaxStep/1e2);
    end
end

%% Error test

% check distance error between lower and upper balljoints
errCalc.FUB = [output.r_P1o_x, output.r_P1o_y, output.r_P1o_z];
errCalc.FLB = [output.r_P3o_x, output.r_P3o_y, output.r_P3o_z];

errCalc.dist = zeros(length(errCalc.FUB(:,1)),1);

% calculate distance between points for entire simulation
for n = 1:length(errCalc.FUB(:,1))
    errCalc.dist(n) = sqrt( ...
        (errCalc.FUB(n,1) - errCalc.FLB(n,1))^2 + ...
        (errCalc.FUB(n,2) - errCalc.FLB(n,2))^2 + ...
        (errCalc.FUB(n,3) - errCalc.FLB(n,3))^2);
end

% find error
errCalc.err = abs(errCalc.dist - errCalc.dist(1));

% plot result
figure("Name","Error check"); hold all; 
title({"Error growth rate check", "King pin length change"});
plot(output.t, errCalc.err); grid minor;
xlabel('sim time (s)');
ylabel('error (mm)');

%% Constraints checking

% angular velocity
Check.w_A = [output.dCamber'; output.dSpin'; output.dToe'];

% translational velocity
Check.v_CP = [output.dWheelbase'; output.dTrack'; output.dTravel'];

% contact patch vector
Check.r_CP = [output.r_CP_O_x'; output.r_CP_O_y'; output.r_CP_O_z'];

% outer PUP vectors (from origin)
Check.r_P1o = [output.r_P1o_x'; output.r_P1o_y'; output.r_P1o_z'];
Check.r_P2o = [output.r_P2o_x'; output.r_P2o_y'; output.r_P2o_z'];
Check.r_P3o = [output.r_P3o_x'; output.r_P3o_y'; output.r_P3o_z'];
Check.r_P4o = [output.r_P4o_x'; output.r_P4o_y'; output.r_P4o_z'];
Check.r_P5o = [output.r_P5o_x'; output.r_P5o_y'; output.r_P5o_z'];

% outer PUP vectors (from contact patch)
Check.r_P1_CP = Check.r_P1o - Check.r_CP;
Check.r_P2_CP = Check.r_P2o - Check.r_CP;
Check.r_P3_CP = Check.r_P3o - Check.r_CP;
Check.r_P4_CP = Check.r_P4o - Check.r_CP;
Check.r_P5_CP = Check.r_P5o - Check.r_CP;

Check.test1 = zeros(numel(Check.w_A(1,:)), 1);
Check.test2 = zeros(numel(Check.w_A(1,:)), 1);
Check.test3 = zeros(numel(Check.w_A(1,:)), 1);
Check.test4 = zeros(numel(Check.w_A(1,:)), 1);
Check.test5 = zeros(numel(Check.w_A(1,:)), 1);

% find veloctity vectors of each point
for n = 1:numel(Check.w_A(1,:))

    % velocity vectors
    Check.v_P1 = Check.v_CP(:,n) + cross(Check.w_A(:,n), Check.r_P1_CP(:,n));
    Check.v_P2 = Check.v_CP(:,n) + cross(Check.w_A(:,n), Check.r_P2_CP(:,n));
    Check.v_P3 = Check.v_CP(:,n) + cross(Check.w_A(:,n), Check.r_P3_CP(:,n));
    Check.v_P4 = Check.v_CP(:,n) + cross(Check.w_A(:,n), Check.r_P4_CP(:,n));
    Check.v_P5 = Check.v_CP(:,n) + cross(Check.w_A(:,n), Check.r_P5_CP(:,n));

    % link vectors
    Check.r_P1_CH = Check.r_P1o(:,n) - PUP.r_P1i;
    Check.r_P2_CH = Check.r_P2o(:,n) - PUP.r_P2i;
    Check.r_P3_CH = Check.r_P3o(:,n) - PUP.r_P3i;
    Check.r_P4_CH = Check.r_P4o(:,n) - PUP.r_P4i;
    Check.r_P5_CH = Check.r_P5o(:,n) - PUP.r_P5i;

    % dot product (should equal zero)
    Check.test1(n) = Check.v_P1'*Check.r_P1_CH;
    Check.test2(n) = Check.v_P2'*Check.r_P2_CH;
    Check.test3(n) = Check.v_P3'*Check.r_P3_CH;
    Check.test4(n) = Check.v_P4'*Check.r_P4_CH;
    Check.test5(n) = Check.v_P5'*Check.r_P5_CH;
end

clear n;

disp(['max constraint offset (P1): ', num2str(round(max(abs(Check.test1)), 3, "significant"))]);
disp(['max constraint offset (P2): ', num2str(round(max(abs(Check.test2)), 3, "significant"))]);
disp(['max constraint offset (P3): ', num2str(round(max(abs(Check.test3)), 3, "significant"))]);
disp(['max constraint offset (P4): ', num2str(round(max(abs(Check.test4)), 3, "significant"))]);
disp(['max constraint offset (P5): ', num2str(round(max(abs(Check.test5)), 3, "significant"))]);