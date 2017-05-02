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

%% stella - r = 57.91/2000; % radius in [m]
rStella = 57.91/2000;
LStella = 0.130175; % length of stella in meters
forceStella = [0.5,1.45,1.96;...
            1.25,1.48,2.28;...
            1.35,1.27,2.31;...
            1.43,1.17,2.11];
RPMStella = [3100,4060,6000;...
            3024,4130,5960;...
            2950,3930,5950;...
            2800,4000,5870]; % convert from [rpm] to [rps]
% expected relationship
RPMMagnusStella = (2800:100:6000); % [rpm]

% FL = 4*pi^2*r^2*rho*v*omega*L
FLStella4 = 4*pi^2*rStella^2*airrho*speed4.*RPMMagnusStella/60*LStella;
FLStella3 = 4*pi^2*rStella^2*airrho*speed3.*RPMMagnusStella/60*LStella;
FLStella2 = 4*pi^2*rStella^2*airrho*speed2.*RPMMagnusStella/60*LStella;
FLStella1 = 4*pi^2*rStella^2*airrho*speed1.*RPMMagnusStella/60*LStella;

% polyfitting data to line
% speed4
stellaFit4 = polyfit(RPMStella(4,:),forceStella(4,:),1);
stellaFit4x = 2800:(5870-2800)/1000:5870;
stellaFit4y = stellaFit4x*stellaFit4(1) + stellaFit4(2);
% speed3
stellaFit3 = polyfit(RPMStella(3,:),forceStella(3,:),1);
stellaFit3x = 2950:(5950-2950)/1000:5950;
stellaFit3y = stellaFit3x*stellaFit3(1) + stellaFit3(2);
% speed2
stellaFit2 = polyfit(RPMStella(2,:),forceStella(2,:),1);
stellaFit2x = 3024:(5960-3024)/1000:5960;
stellaFit2y = stellaFit2x*stellaFit2(1) + stellaFit2(2);
% speed1
stellaFit1 = polyfit(RPMStella(1,:),forceStella(1,:),1);
stellaFit1x = 3100:(6000-3100)/1000:6000;
stellaFit1y = stellaFit1x*stellaFit1(1) + stellaFit1(2);

% speed4 plots
figure;
hold on;
plot(RPMMagnusStella,FLStella4,'r--');
plot(RPMStella(4,:),forceStella(4,:),'bx',stellaFit4x,stellaFit4y,'b');
xlabel('Cylinder Rotation Speed [RPM]','FontSize',12);
ylabel('Lift Force (N)','FontSize',12);
title('Lift Force vs. RPM at 24 m/s','FontSize',14);
l = legend('Magnus Predicted','Experiment Data','Experiment Fit','Location','best');
set(l,'FontSize',12);

% speed3 plots
figure;
hold on;
plot(RPMMagnusStella,FLStella3,'r--');
plot(RPMStella(3,:),forceStella(3,:),'bx',stellaFit3x,stellaFit3y,'b');
xlabel('Cylinder Rotation Speed [RPM]','FontSize',12);
ylabel('Lift Force (N)','FontSize',12);
title('Lift Force vs. RPM at 20 m/s','FontSize',14);
l = legend('Magnus Predicted','Experiment Data','Experiment Fit','Location','best');
set(l,'FontSize',12);

% speed2 plots
figure;
hold on;
plot(RPMMagnusStella,FLStella2,'r--');
plot(RPMStella(2,:),forceStella(2,:),'bx',stellaFit2x,stellaFit2y,'b');
xlabel('Cylinder Rotation Speed [RPM]','FontSize',12);
ylabel('Lift Force (N)','FontSize',12);
title('Lift Force vs. RPM at 16 m/s','FontSize',14);
l = legend('Magnus Predicted','Experiment Data','Experiment Fit','Location','best');
set(l,'FontSize',12);

% speed1 plots
figure;
hold on;
plot(RPMMagnusStella,FLStella1,'r--');
plot(RPMStella(1,:),forceStella(1,:),'bx',stellaFit1x,stellaFit1y,'b');
xlabel('Cylinder Rotation Speed [RPM]','FontSize',12);
ylabel('Lift Force (N)','FontSize',12);
title('Lift Force vs. RPM at 12 m/s','FontSize',14);
l = legend('Magnus Predicted','Experiment Data','Experiment Fit','Location','best');
set(l,'FontSize',12);

%% Bud - r = 83.82/2000; % radius in [m]
rBud = 83.82/2000; % finding radius from diamter in [mm] to [m]
LBud = 0.1762125; % length of bud cylinder in meters
forceBud = [2.3,3.17,1.65;...
            0,0,0;...
            10.13,21.92,13.91;...
            10.25,13.11,8.97];
RPMBud = [3200,4940,5800;...
        0,0,0;...
        3291,4700,5844;...
        3100,4860,5916]; % convert from [rpm] to [rps]
% expected relationship
RPMMagnusBud = (2500:100:6000); % [rpm]

% FL = 4*pi^2*r^2*rho*v*omega*L
FLBud4 = 4*pi^2*rBud^2*airrho*speed4.*RPMMagnusBud/60*LBud;
FLBud3 = 4*pi^2*rBud^2*airrho*speed3.*RPMMagnusBud/60*LBud;
FLBud2 = 4*pi^2*rBud^2*airrho*speed2.*RPMMagnusBud/60*LBud;
FLBud1 = 4*pi^2*rBud^2*airrho*speed1.*RPMMagnusBud/60*LBud;

% polyfitting data to line
% speed4
budFit4 = polyfit(RPMBud(4,:),forceBud(4,:),1);
budFit4x = 3100:(5916-3100)/1000:5916;
budFit4y = budFit4x*budFit4(1) + budFit4(2);
% speed3
budFit3 = polyfit(RPMBud(3,:),forceBud(3,:),1);
budFit3x = 3291:(5844-3291)/1000:5844;
budFit3y = budFit3x*budFit3(1) + budFit3(2);
% speed1
budFit1 = polyfit(RPMBud(1,:),forceBud(1,:),1);
budFit1x = 3200:(5800-3200)/1000:5800;
budFit1y = budFit1x*budFit1(1) + budFit1(2);

% speed4 plots
figure;
hold on;
plot(RPMMagnusBud,FLBud4,'r--');
plot(RPMBud(4,:),forceBud(4,:),'bx',budFit4x,budFit4y,'b');
xlabel('Cylinder Rotation Speed [RPM]','FontSize',12);
ylabel('Lift Force (N)','FontSize',12);
title('Lift Force vs. RPM at 24 m/s','FontSize',14);
l = legend('Magnus Predicted','Experiment Data','Experiment Fit','Location','best');
set(l,'FontSize',12);

% speed3 plots
figure;
hold on;
plot(RPMMagnusBud,FLBud3,'r--');
plot(RPMBud(3,:),forceBud(3,:),'bx',budFit3x,budFit3y,'b');
xlabel('Cylinder Rotation Speed [RPM]','FontSize',12);
ylabel('Lift Force (N)','FontSize',12);
title('Lift Force vs. RPM at 20 m/s','FontSize',14);
l = legend('Magnus Predicted','Experiment Data','Experiment Fit','Location','best');
set(l,'FontSize',12);

% speed2 plot for Bud does not exist

% speed1 plots
figure;
hold on;
plot(RPMMagnusBud,FLBud1,'r--');
plot(RPMBud(1,:),forceBud(1,:),'bx',budFit1x,budFit1y,'b');
xlabel('Cylinder Rotation Speed [RPM]','FontSize',12);
ylabel('Lift Force (N)','FontSize',12);
title('Lift Force vs. RPM at 12 m/s','FontSize',14);
l = legend('Magnus Predicted','Experiment Data','Experiment Fit','Location','best');
set(l,'FontSize',12);

%% Quaker - r = 128.27/2000; % radius in [m]
rQuaker = 128.27/2000;
LQuaker = 0.2286; % length of quaker oats cylinder in meters
forceQuaker = [3.72,3.14,4.35;...
            6,4.97,5.26;...
            8.42,7.25,7.67;...
            10.6,15.5,10.43];
RPMQuaker = [3720,4560,5560;...
            3660,4470,5630;...
            3675,4050,5500;...
            3000,3160,5200]; % convert from [rpm] to [rps]
% expected relationship
RPMMagnusQuaker = (2500:100:6000); % [rpm]

% FL = 4*pi^2*r^2*rho*v*omega*L
FLQuaker4 = 4*pi^2*rQuaker^2*airrho*speed4.*RPMMagnusQuaker/60*LQuaker;
FLQuaker3 = 4*pi^2*rQuaker^2*airrho*speed3.*RPMMagnusQuaker/60*LQuaker;
FLQuaker2 = 4*pi^2*rQuaker^2*airrho*speed2.*RPMMagnusQuaker/60*LQuaker;
FLQuaker1 = 4*pi^2*rQuaker^2*airrho*speed1.*RPMMagnusQuaker/60*LQuaker;

% polyfitting data to line
% speed4
quakerFit4 = polyfit(RPMQuaker(4,:),forceQuaker(4,:),1);
quakerFit4x = 3000:(5200-3000)/1000:5200;
quakerFit4y = quakerFit4x*quakerFit4(1) + quakerFit4(2);
% speed3
quakerFit3 = polyfit(RPMQuaker(3,:),forceQuaker(3,:),1);
quakerFit3x = 3675:(5500-3675)/1000:5500;
quakerFit3y = quakerFit3x*quakerFit3(1) + quakerFit3(2);
% speed2
quakerFit2 = polyfit(RPMQuaker(2,:),forceQuaker(2,:),1);
quakerFit2x = 3660:(5630-3660)/1000:5630;
quakerFit2y = quakerFit2x*quakerFit2(1) + quakerFit2(2);
% speed1
quakerFit1 = polyfit(RPMQuaker(1,:),forceQuaker(1,:),1);
quakerFit1x = 3720:(5560-3720)/1000:5560;
quakerFit1y = quakerFit1x*quakerFit1(1) + quakerFit1(2);

% speed4 plots
figure;
hold on;
plot(RPMMagnusQuaker,FLQuaker4,'r--');
plot(RPMQuaker(4,:),forceQuaker(4,:),'bx',quakerFit4x,quakerFit4y,'b');
xlabel('Cylinder Rotation Speed [RPM]','FontSize',12);
ylabel('Lift Force (N)','FontSize',12);
title('Lift Force vs. RPM at 24 m/s','FontSize',14);
l = legend('Magnus Predicted','Experiment Data','Experiment Fit','Location','best');
set(l,'FontSize',12);

% speed3 plots
figure;
hold on;
plot(RPMMagnusQuaker,FLQuaker3,'r--');
plot(RPMQuaker(3,:),forceQuaker(3,:),'bx',quakerFit3x,quakerFit3y,'b');
xlabel('Cylinder Rotation Speed [RPM]','FontSize',12);
ylabel('Lift Force (N)','FontSize',12);
title('Lift Force vs. RPM at 20 m/s','FontSize',14);
l = legend('Magnus Predicted','Experiment Data','Experiment Fit','Location','best');
set(l,'FontSize',12);

% speed2 plots
figure;
hold on;
plot(RPMMagnusQuaker,FLQuaker2,'r--');
plot(RPMQuaker(2,:),forceQuaker(2,:),'bx',quakerFit2x,quakerFit2y,'b');
xlabel('Cylinder Rotation Speed [RPM]','FontSize',12);
ylabel('Lift Force (N)','FontSize',12);
title('Lift Force vs. RPM at 16 m/s','FontSize',14);
l = legend('Magnus Predicted','Experiment Data','Experiment Fit','Location','best');
set(l,'FontSize',12);

% speed1 plots
figure;
hold on;
plot(RPMMagnusQuaker,FLQuaker1,'r--');
plot(RPMQuaker(1,:),forceQuaker(1,:),'bx',quakerFit1x,quakerFit1y,'b');
xlabel('Cylinder Rotation Speed [RPM]','FontSize',12);
ylabel('Lift Force (N)','FontSize',12);
title('Lift Force vs. RPM at 12 m/s','FontSize',14);
l = legend('Magnus Predicted','Experiment Data','Experiment Fit','Location','best');
set(l,'FontSize',12);

%% comparison to theory completed, looking for patterns within the collected
% data
% plot of just the experimental data
% quaker oats cylinder
figure;
hold on;
plot(RPMQuaker(4,:),forceQuaker(4,:),'--*');
plot(RPMQuaker(3,:),forceQuaker(3,:),'--x');
plot(RPMQuaker(2,:),forceQuaker(2,:),'--d');
plot(RPMQuaker(1,:),forceQuaker(1,:),'--s');
grid on;
xlabel('Rotation Speed (RPM)','FontSize',12);
ylabel('Life Force (N)','FontSize',12);
title({'Measured Lift Force of Quaker Oats Cylinder vs. RPM','at Constant Wind Speeds'},'FontSize',14);
l = legend('Location','best','24 m/s','20 m/s','16 m/s','12 m/s');
set(l,'FontSize',12);

% Bud Heavy cylinder
figure;
hold on;
plot(RPMBud(4,:),forceBud(4,:),'--*');
plot(RPMBud(3,:),forceBud(3,:),'--x');
plot(RPMBud(1,:),forceBud(1,:),'--s');
grid on;
xlabel('Rotation Speed (RPM)','FontSize',12);
ylabel('Life Force (N)','FontSize',12);
title({'Measured Lift Force of Bud Heavy Cylinder vs. RPM','at Constant Wind Speeds'},'FontSize',14);
l = legend('Location','best','24 m/s','20 m/s','12 m/s');
set(l,'FontSize',12);
text(3250,22.5,'16 m/s wind speed has no data','FontSize',12);

% Quaker cylinder
figure;
hold on;
plot(RPMQuaker(4,:),forceQuaker(4,:),'--*');
plot(RPMQuaker(3,:),forceQuaker(3,:),'--x');
plot(RPMQuaker(2,:),forceQuaker(2,:),'--d');
plot(RPMQuaker(1,:),forceQuaker(1,:),'--s');
grid on;
xlabel('Rotation Speed (RPM)','FontSize',12);
ylabel('Life Force (N)','FontSize',12);
title({'Measured Lift Force of Quaker Cylinder vs. RPM','at Constant Wind Speeds'},'FontSize',14);
l = legend('Location','best','24 m/s','20 m/s','16 m/s','12 m/s');
set(l,'FontSize',12);

% %% 2. Effect of changing wind speed on lift force
% % Experiment was conducted to have approximately the same RPM. Assuming the
% % difference in lift force from <300 RPM difference is negligible for the
% % purpose of seeking a pattern.
% speeds = [speed4,speed3,speed2,speed1];
% 
% % Stella Cylinder
% figure;
% hold on;
% plot(speeds,forceStella(:,1),'--*');
% plot(speeds,forceStella(:,2),'-.x');
% plot(speeds,forceStella(:,3),'k:d');
% xlim([10 26]);
% xlabel('Wind Speeds (m/s)');
% ylabel('Lift Force (N)');
% title('Effect of Wind Speed on Stella Cylinder Lift Force Assuming Constant RPM');
% legend('Location','best','~3000 RPM','~4000 RPM','~6000 RPM');
% 
% % Bud Cylinder
% figure;
% hold on;
% plot(speeds([1,2,4]),forceBud([1,2,4],1),'--*');
% plot(speeds([1,2,4]),forceBud([1,2,4],2),'-.x');
% plot(speeds([1,2,4]),forceBud([1,2,4],3),'k:d');
% xlim([10 26]);
% xlabel('Wind Speeds (m/s)');
% ylabel('Lift Force (N)');
% title('Effect of Wind Speed on Bud Cylinder Lift Force Assuming Constant RPM');
% legend('Location','best','~3000 RPM','~5000 RPM','~6000 RPM');
% 
% % Oats Cylinder
% figure;
% hold on;
% plot(speeds,forceQuaker(:,1),'--*');
% plot(speeds,forceQuaker(:,2),'-.x');
% plot(speeds,forceQuaker(:,3),'k:d');
% xlim([10 26]);
% xlabel('Wind Speeds (m/s)');
% ylabel('Lift Force (N)');
% title('Effect of Wind Speed on Oats Cylinder Lift Force Assuming Constant RPM');
% legend('Location','best','~3500 RPM','~4250 RPM','~5500 RPM');
% 
% %% 3. Effect of radius on lift force - After completing previous two sections
% % wind speed 23.8 m/s
% d1 = 57.91/1000; % stella diameter in meters
% d2 = 83.82/1000; % budweiser diameter in meters
% d3 = 128.27/1000; % quaker oats diameter in meters
% 
% % lift force of stella diameter at 23.8 wind speed at around 3000 RPM
% fl1_v238_R3000 = 1.43; % [N]
% fl2_v238_R3000 = 10.25; % [N]
% fl3_v238_R3000 = 10.6; % [N]