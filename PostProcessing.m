%% Prepare workspace

clear; clc; close all;

%% Load data

load("dataED_with_steer.mat");

% find zero steer angle result
idx_f = find(dataED.Front.gamma_deg == 0);
idx_r = find(dataED.Rear.gamma_deg == 0);

front.X = dataED.Front.wheelX(:,idx_f);
front.Y = dataED.Front.wheelY(:,idx_f);
front.Z = dataED.Front.wheelZ(:,idx_f);

rear.X = dataED.Rear.wheelX(:,idx_r);
rear.Y = dataED.Rear.wheelY(:,idx_r);
rear.Z = dataED.Rear.wheelZ(:,idx_r);

front.camber = dataED.Front.camber(:,idx_f);
front.toe    = dataED.Front.toe(:,idx_f);

rear.camber = dataED.Rear.camber(:,idx_r);
rear.toe    = dataED.Rear.toe(:,idx_r);

% remove unused data
clear dataED;

%% Plot wheel path
figure("Name", "wheel path"); 
subplot(1,2,1); hold all;
    title('Front');
    plot3(front.X, front.Y, front.Z);
    box on; grid minor; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(45,30);
subplot(1,2,2); hold all;
    title('Rear');
    plot3(rear.X, rear.Y, rear.Z);
    box on; grid minor; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(45,30);

figure("Name", "camber & toe change"); 
subplot(1,2,1); hold all;
plot(front.Z, front.camber);
plot(rear.Z, rear.camber);
box on; grid minor;
subplot(1,2,2); hold all;
plot(front.Z, front.toe);
plot(rear.Z, rear.toe);
box on; grid minor;

%% Resample values to obtain constant dz
% 
% % sample distance
% dz = 0.5;
% 
% % front
% sp_x_f = spline(front.Z, front.X);
% sp_y_f = spline(front.Z, front.Y);
% 
% % rear
% sp_x_r = spline(rear.Z, rear.X);
% sp_y_r = spline(rear.Z, rear.Y);
% 
% % resample
% front.Z = ceil(min(front.Z)):dz:floor(max(front.Z));
% rear.Z = ceil(min(rear.Z)):dz:floor(max(rear.Z));
% 
% front.X = fnval(sp_x_f, front.Z);
% front.Y = fnval(sp_y_f, front.Z);
% rear.X = fnval(sp_x_r, rear.Z);
% rear.Y = fnval(sp_y_r, rear.Z);

%% Anti roll angles and RC height

sp_lat_f = spline(front.Z, front.Y);

sp_lat_f_der = fnder(sp_lat_f,1);

front.theta_lat = fnval(sp_lat_f_der,front.Z);

front.h_RC = 625.*front.theta_lat;

figure; hold all
plot(front.Z, front.h_RC);
xline(0, 'k-');
box on; grid minor;

%% Camber gain and FVSA length

sp_camber_f = spline(front.Z, front.camber);
sp_camber_r = spline(rear.Z, rear.camber);

sp_camber_der_f = fnder(sp_camber_f, 1);
sp_camber_der_r = fnder(sp_camber_r, 1);

front.cambergain = fnval(sp_camber_der_f, front.Z);
rear.cambergain = fnval(sp_camber_der_r, rear.Z);

front.FVSA = -1./front.cambergain;
rear.FVSA = -1./rear.cambergain;

figure("Name", "cambergain & FVSA"); 
subplot(1,2,1); hold all;
plot(front.Z, front.cambergain*180/pi);
plot(rear.Z, rear.cambergain*180/pi);
box on; grid minor;
subplot(1,2,2); hold all;
plot(front.Z, front.FVSA);
plot(rear.Z, rear.FVSA);
box on; grid minor;

%% ANYTHING AFTER THIS IS PROBABLY WRONG
for i = 2:length(z_new)-1
    dgammadz(i) = 1/(dz^2)*(y_new(i+1) - 2*y_new(i) + y_new(i-1))*180/pi;
end
dgammadz(1) = [];

figure; hold all;
plot(dgammadz, z_new(2:end-1));
yline(0, '-k');
xlabel('camber gain (deg/mm)');
ylabel('wheel travel (mm)');
box on; grid minor;

L_FVSA = 1./(dgammadz*pi/180);

%%

gammaA = zeros(length(z_new)-3,1);
gammaB = zeros(length(z_new)-3,1);

% numerical integration to find camber
for i = 3:length(z_new)-3
    gammaA(i) = gammaA(i-1) + (dz)/6*(dgammadz(i-1) + 4*dgammadz(i) + dgammadz(i+1));
    gammaB(i) = gammaB(i-1) + dgammadz(i-1)*dz;
end

% correct for static camber
idx = find(z_new == min(abs(z_new)));
gammaA = gammaA - gammaA(idx);
gammaA(1:2) = [];
gammaB = gammaB - gammaB(idx);
gammaB(1:2) = [];


figure; hold all;
plot(gammaA, z_new(3:end-3));
plot(gammaB, z_new(3:end-3));
xline(0, 'k-'); yline(0, 'k-');
box on; grid minor;
xlabel('camber (deg)'); ylabel('travel (mm)');

plot(front.Z, front.camber, '*');