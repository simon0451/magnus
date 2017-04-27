% Final Project
% Magnus Effect on a Cylindrical Airfoil
% Team Members
% Jesse Feng, Simon Popecki, James Skinner
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Expected Analytical Data
close all;clear all;
% Assuming Laminar Flow
% drag = .5*CD*rhoair*A*v*(-v) idk what either v are
% 25 mm ~ 1 inch
radius = 250; % mm
A = pi*radius.^2; % mm^2 Area
radiusm = radius./1000; % m
Am = A./1000000; % m^2 Area
rhoair = 1.2754; % kg/m^3
% % V = sqrt(2(Pt-P)/rhoair)
% Vwind = sqrt((2*Pdynamic)/rhoair); % m/s
% Will still need pressure readings from pitot tube in wind tunnel
Vwind = 25; % m/s
rpm = 100; % per minute
rps = rpm/60; % per second
w = rps*2*pi; % rad/sec
G = 2*pi.*(radiusm.^2)*w;
lift = rhoair*Vwind*(2*pi*w.*(radiusm.^2));
CL = 2*lift/(rhoair*(Vwind.^2)*Am); % Coefficient of lift

%CD = 2*drag/(rhoair*(Vwind.^2).*Am); % Coefficient of drag