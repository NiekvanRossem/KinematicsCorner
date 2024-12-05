%% Prepare workspace

% clear everything
clear; close all; clc;

% set all figures to docked mode
set(0,'DefaultFigureWindowStyle','docked');

%% Input settings

% set wheel travel vertical speed
dTravel = 1;

% settings
Settings.Axle = "Front";
Settings.Distr = 'Uniform';
Settings.AnalysisLength = 1e4;

% set uncertainty
u = 0.10;

%% Load suspension hardpoints

% load suspension hardpoints
[Car, PUP] = DUT24_PUP(Settings);

% find loaded radius
Car.RL = abs(PUP.r_WC_O(3) - PUP.r_CP_O(3));

%% Monte Carlo analysis

for n = 1:Settings.AnalysisLength

    % create state vector (including uncertainty)
    r = vertcat( ...
        PUP.r_P1o + u*2*(rand(3,1)-0.5), ...
        PUP.r_P2o + u*2*(rand(3,1)-0.5), ...
        PUP.r_P3o + u*2*(rand(3,1)-0.5), ...
        PUP.r_P4o + u*2*(rand(3,1)-0.5), ...
        PUP.r_P5o + u*2*(rand(3,1)-0.5), ...
        PUP.r_P1i + u*2*(rand(3,1)-0.5), ...
        PUP.r_P2i + u*2*(rand(3,1)-0.5), ...
        PUP.r_P3i + u*2*(rand(3,1)-0.5), ...
        PUP.r_P4i + u*2*(rand(3,1)-0.5), ...
        PUP.r_P5i + u*2*(rand(3,1)-0.5), ...
        PUP.r_CP_O, ...
        dTravel);
    
    [~, q] = FiveLink(0, r);
    
    % generalised DoF
    output.dWheelbase   = q(:,1);
    output.dTrack       = q(:,2);
    output.dTravel      = q(:,6);
    output.dCamber      = q(:,3);
    output.dSpin        = q(:,4);
    output.dToe         = q(:,5);
    output.Track        = 2*PUP.r_CP_O(2);
    
    % calculate equivalent roll rate
    output.dRoll  = 180/pi*atan(output.dTravel./(0.5*output.Track));
    
    % Suspension params
    output.RollCentre(n)   = output.Track./2.*(output.dTrack./output.dTravel);     % mm
    output.CamberGainB(n)  = -output.dCamber./output.dTravel*180/pi;               % deg/mm
    output.BumpSteer(n)    = output.dToe./output.dTravel*180/pi;                   % deg/mm
    output.CasterGain(n)   = -output.dSpin./output.dTravel*180/pi;                 % deg/mm
    output.AntiRoll(n)     = atan(output.dTrack./output.dTravel)*180/pi;           % deg
    output.AntiPitch(n)    = atan(-output.dWheelbase./output.dTravel)*180/pi;      % deg
    output.LatGrad(n)      = output.dTrack./output.dTravel;                        % mm/mm
    output.LongGrad(n)     = output.dWheelbase./output.dTravel;                    % mm/mm
    output.CamberGainR(n)  = output.CamberGainB(n).*0.5.*output.Track.*sind(1)+1;     % deg/deg
    output.RollSteer(n)    = 180/pi*output.dToe./output.dRoll;                      % deg/deg

end

%%
figure("Name", "Roll centre distribution");
lower = round(min(output.RollCentre), 1);
upper = round(max(output.RollCentre), 1);
sgtitle({'Probability distribution of roll centre height', ['tolerance = ', num2str(u), ' mm']});
histogram(output.RollCentre, linspace(lower, upper, 50), 'Normalization','probability');
xlabel('Roll centre height (mm)'); ylabel('Probability');
xlim([lower upper]);
box on; grid minor;

figure("Name", "Bumpsteer distribution");
lower = round(min(output.BumpSteer), 5);
upper = round(max(output.BumpSteer), 5);
sgtitle({'Probability distribution of bump steer gradient', ['tolerance = ', num2str(u), ' mm']});
histogram(output.BumpSteer, linspace(lower, upper, 50), 'Normalization','probability');
xlabel('Bumpsteer (deg/mm)'); ylabel('Probability');
xlim([lower upper]);
box on; grid minor;

figure("Name", "Camber gain distribution");
lower = round(min(output.CamberGainB), 4);
upper = round(max(output.CamberGainB), 4);
sgtitle({'Probability distribution of camber gain', ['tolerance = ', num2str(u), ' mm']});
histogram(output.CamberGainB, linspace(lower, upper, 50), 'Normalization','probability');
xlabel('Camber gain (deg/mm)'); ylabel('Probability');
xlim([lower upper]);
box on; grid minor;

figure("Name", "Camber gain distribution");
lower = round(min(output.CamberGainB), 4);
upper = round(max(output.CamberGainB), 4);
sgtitle({'Probability distribution of camber gain', ['tolerance = ', num2str(u), ' mm']});
histogram(output.CamberGainB, linspace(lower, upper, 50), 'Normalization','probability');
xlabel('Camber gain (deg/mm)'); ylabel('Probability');
xlim([lower upper]);
box on; grid minor;

%% Print output

disp(' ~~ Results ~~ ');
disp(['Roll centre height = ', num2str(round(mean(output.RollCentre),2)), ' ± ', num2str(round(std(output.RollCentre),2)), ' mm']);