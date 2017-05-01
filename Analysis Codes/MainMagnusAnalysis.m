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
% FL = rho*v*G*L = rho*v*2*pi*r^2*omega*L = rho*v*4*pi*r^3*omega
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

%% Overall experiment variables:
% air density:
% Durham atmospheric pressure on APril 30, 2017
% data obtained from WeatherForYou.com for April 30, 2017
atmPressure = 30.22; % [inHg]
atmP = 30.22*3386.39; % converting from [inHg] to [Pa]
air_rho = 

% windspeeds:
% pitot tube readings
p1_4Speed = 3; % [inWater]
p2_4Speed = 1.6;
% dynamic pressure = difference of two
pDyn_4Speed = p1_4Speed - p2_4Speed;
% finding wind speed from dynamic pressure
v_4Speed = sqrt(2*pDyn_4Speed/air_rho);

p1_3Speed = 2.45;
p2_3Speed = 1.5;

p1_2Speed = 2;
p2_2Speed = 1.35;

p1_1Speed = 1.65;
p2_1Speed = 1.3;

%% 1. Effect of RPM on lift force at constant wind speed for each airfoil

% stella - r = 57.91/2000; % radius in [m]
rstella = 57.91/2000;



%% 3. Effect of radius on lift force - After completing previous two sections
% wind speed 23.8 m/s
d1 = 57.91/1000; % stella diameter in meters
d2 = 83.82/1000; % budweiser diameter in meters
d3 = 128.27/1000; % quaker oats diameter in meters

% lift force of stella diameter at 23.8 wind speed at around 3000 RPM
fl1_v238_R3000 = 1.43; % [N]
fl2_v238_R3000 = 10.25; % [N]
fl3_v238_R3000 = 10.6; % [N]