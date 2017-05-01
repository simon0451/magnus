clear all;
close all;

% Magnus Effect Overview
% Variables:
% FL: lift force - measured by force balance
% rho: air density - calculated from the day's atmospheric pressure
% v: air speed - calculated using pitot tube
% G: vortex strength - found using G = 2*pi*r^2*omega
%                                   r: cylinder radius
%                                   omega: RPM in rad/s
% L: characteristic length of the cylinder = diameter = 2*r
% FL = rho*v*G*L = rho*v*4*pi^2*r^2*omega*L = rho*v*4*pi^2*r^2*omega*L
% 
% Topics to analyze:
% 1. Effect of changing RPM on lift force at constant wind speed
%       - expect linear relationship
% 2. Effect of changing wind speed on lift force at constant RPM
%       - expect linear relationship
% 3. Effect of changing radius on lift force
%       - expect cubic relationship
% 4. Effect of linearly increasing wind speed on lift force
%       - expect linear relationship

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Authors: Feng, Zhangxi; Popecki, Simon; Skinner, James %
% Course: ME646                                          %
% Project: Cylindrical Airfoil Magnus Effect             %
% Date: May 1, 2017                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Overall experiment variables
% See attached excel sheet for the experiment data
% windspeeds from pitot tube readings:
speed4 = 24.01946618; % [m/s]
speed3 = 19.78614266;
speed2 = 16.36650742;
speed1 = 12.00973309;
airrho = 1.2093;
% [kg/m^3] from ideal gas law and the barometric pressure of the day

%% 1. Effect of RPM on lift force at constant wind speed for each airfoil

% stella - r = 57.91/2000; % radius in [m]
rStella = 57.91/2000;
LStella = 0.130175; % length of stella in meters
forceStella = [1.43,1.17,2.11;...
            1.35,1.27,2.31;...
            1.25,1.48,2.28;...
            0.5,1.45,1.96];
sStella = [2800,4000,5870;...
            2950,3930,5950;...
            3024,4130,5960;...
            3100,4060,6000]/60; % convert from [rpm] to [rps]
% expected relationship
sMagnusStella = (2000:100:7000)/60; % from [rpm] to [rps]
CalcPlotFvRPM(rStella,sMagnusStella,sStella,forceStella,LStella);

% Bud - r = 83.82/2000; % radius in [m]
rBud = 83.82/2000; % finding radius from diamter in [mm] to [m]
LBud = 0.1762125; % length of bud cylinder in meters
forceBud = [10.25,13.11,8.97;...
            10.13,21.92,13.91;...
            0,0,0;...
            2.3,3.17,1.65];
sBud = [3100,4860,5916;...
        3291,4700,5844;...
        0,0,0;...
        3200,4940,5800]/60; % convert from [rpm] to [rps]
% expected relationship
sMagnusBud = (2500:100:6000)/60; % in [rps]
CalcPlotFvRPM(rBud,sMagnusBud,sBud,forceBud,LBud);

% Quaker - r = 128.27/2000; % radius in [m]
rQuaker = 128.27/2000;
LQuaker = 0.2286; % length of quaker oats cylinder in meters
forceQuaker = [10.6,15.5,10.43;...
            8.42,7.25,7.67;...
            6,4.97,5.26;...
            3.72,3.14,4.35];
sQuaker = [3000,3160,5200;...
            3675,4050,5500;...
            3660,4470,5630;...
            3720,4560,5560]/60; % convert from [rpm] to [rps]
% expected relationship
sMagnusQuaker = (2500:100:6000)/60; % in [rps]
CalcPlotFvRPM(rQuaker,sMagnusQuaker,sQuaker,forceQuaker,LQuaker);

%% 2. Effect of changing wind speed on lift force


%% 3. Effect of radius on lift force - After completing previous two sections
% wind speed 23.8 m/s
d1 = 57.91/1000; % stella diameter in meters
d2 = 83.82/1000; % budweiser diameter in meters
d3 = 128.27/1000; % quaker oats diameter in meters

% lift force of stella diameter at 23.8 wind speed at around 3000 RPM
fl1_v238_R3000 = 1.43; % [N]
fl2_v238_R3000 = 10.25; % [N]
fl3_v238_R3000 = 10.6; % [N]