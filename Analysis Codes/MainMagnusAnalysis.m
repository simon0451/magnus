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
% L: length of the cylinder
% FL = rho*v*G*L = rho*v*4*pi^2*r^2*omega*L = rho*v*4*pi^2*r^2*omega*L
% 
% Topics to analyze:
% 1. Effect of changing RPM on lift force at constant wind speed
%       - expect linear relationship
% 2. Effect of changing wind speed on lift force at constant RPM
%       - expect linear relationship
% 3. Effect of linearly increasing wind speed on lift force
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

%% 2. Effect of changing wind speed on lift force

%% Calculating Reynolds number for 3 cylinders and all 4 speeds 

speeds1to4 = [speed1,speed2,speed3,speed4];
nu = 15.11e-6; % (m^2/s) kinematic viscosity of air at room temperature

ReStella = speeds1to4.*(rStella*2)/nu;
ReBud = speeds1to4.*(rBud*2)/nu;
ReQuaker = speeds1to4.*(rQuaker*2)/nu;
strReStella = sprintf('Re = %1.3d \n',ReStella);
strReBud = sprintf('Re = %1.3d \n',ReBud);
strReQuaker = sprintf('Re = %1.3d \n',ReQuaker);
disp('Stella Reynolds Number for velocities in increasing magnitude (12,16,20,24) m/s')
disp(strReStella)
disp('Bud Heavy Reynolds Number for velocities in increasing magnitude (12,16,20,24) m/s')
disp(strReBud)
disp('Quaker Reynolds Number for velocities in increasing magnitude (12,16,20,24) m/s')
disp(strReQuaker)


%% Plotting all the data
% Experiment was conducted to have approximately the same RPM. Assuming the
% difference in lift force from <300 RPM difference is negligible for the
% purpose of seeking a pattern.

speeds = [speed4,speed3,speed2,speed1];

% Stella Cylinder
figure;
hold on;
plot(speeds,forceStella(:,3),'--*');
plot(speeds,forceStella(:,2),'-.x');
plot(speeds,forceStella(:,1),'k:d');
xlim([10 26]);
xlabel('Wind Speeds (m/s)','FontSize',12);
ylabel('Lift Force (N)','FontSize',12);
title({'Effect of Wind Speed on Stella Cylinder Lift Force','Assuming Constant RPM'},'FontSize',14);
l = legend('Location','best','~3000 RPM','~4000 RPM','~6000 RPM');
set(l,'FontSize',12);

% Bud Cylinder
figure;
hold on;
plot(speeds([1,3,4]),forceBud([1,3,4],3),'--*');
plot(speeds([1,3,4]),forceBud([1,3,4],2),'-.x');
plot(speeds([1,3,4]),forceBud([1,3,4],1),'k:d');
xlim([10 26]);
xlabel('Wind Speeds (m/s)','FontSize',12);
ylabel('Lift Force (N)','FontSize',12);
title({'Effect of Wind Speed on Bud Cylinder Lift Force','Assuming Constant RPM'},'FontSize',14);
l = legend('Location','best','~3000 RPM','~5000 RPM','~6000 RPM');
set(l,'FontSize',12);

% Oats Cylinder
figure;
hold on;
plot(speeds,forceQuaker(:,3),'--*');
plot(speeds,forceQuaker(:,2),'-.x');
plot(speeds,forceQuaker(:,1),'k:d');
xlim([10 26]);
xlabel('Wind Speeds (m/s)','FontSize',12);
ylabel('Lift Force (N)','FontSize',12);
title({'Effect of Wind Speed on Oats Cylinder Lift Force','Assuming Constant RPM'},'FontSize',14);
l = legend('Location','best','~3500 RPM','~4250 RPM','~5500 RPM');
set(l,'FontSize',12);

%% fitting the wind speed data
stellaWindRPMfit6k = polyfit(speeds',forceStella(:,1),1);
sWRfitx = speeds(4):(speeds(1) - speeds(4))/10:speeds(1);
sWRfity = sWRfitx*stellaWindRPMfit6k(1) + stellaWindRPMfit6k(2);

figure;
plot(speeds,forceStella(:,1),'*',sWRfitx,sWRfity);
xlabel('Wind Speed (m/s)','FontSize',12);
ylabel('Lift Force (N)','FontSize',12);
title('Lift Force vs. Wind Speed Stella Cylinder at 6000 rpm','FontSize',14);
l = legend('Location','best','Data','Best Fit');
set(l,'FontSize',12);

budWindRPMfit6k = polyfit(speeds([1,3,4])',forceBud([1,3,4],1),1);
bWRfitx = speeds(4):(speeds(1) - speeds(4))/10:speeds(1);
bWRfity = bWRfitx*budWindRPMfit6k(1) + budWindRPMfit6k(2);

figure;
plot(speeds([1,3,4]),forceBud([1,3,4],1),'*',bWRfitx,bWRfity);
xlabel('Wind Speed (m/s)','FontSize',12);
ylabel('Lift Force (N)','FontSize',12);
title('Lift Force vs. Wind Speed Bud Cylinder 6000 rpm','FontSize',14);
l = legend('Location','best','Data','Best Fit');
set(l,'FontSize',12);

quakerWindRPMfit6k = polyfit(speeds',forceQuaker(:,1),1);
qWRfitx = speeds(4):(speeds(1) - speeds(4))/10:speeds(1);
qWRfity = qWRfitx*quakerWindRPMfit6k(1) + quakerWindRPMfit6k(2);

figure;
plot(speeds,forceQuaker(:,1),'*',qWRfitx,qWRfity);
xlabel('Wind Speed (m/s)','FontSize',12);
ylabel('Lift Force (N)','FontSize',12);
title('Lift Force vs. Wind Speed Quaker Cylinder 6000 rpm','FontSize',14);
l = legend('Location','best','Data','Best Fit');
set(l,'FontSize',12);

%% 3. Effect of increasing wind speed on a fixed RPM on the lift force

%% Plotting and fitting
windSpeed = [11.12,16.37,20.3,23.59,26.47,29.42]; % [m/s]
liftForce = [0.67,0.61,0.5,0.4,0.36,0.47]; % [N]

windForceFit = polyfit(windSpeed,liftForce,1);
windForcex = linspace(windSpeed(1),windSpeed(end),6);
windForcey = windForcex*windForceFit(1) + windForceFit(2);

%% Error Analysis
% confidence interval-------------------------------------------
% defining the degree of freedom and confidence interval
N = length(windSpeed);
dF = N - (1 + 1);
confidenceLevel = 0.975; % Matlab uses one-tail probability

% finding the t score
t = tinv(confidenceLevel,dF);
% t = 2.3060

% average value of x axis data
xMean = mean(windSpeed);

% the sum of difference of X - xbar squared
SSxx = sum((windSpeed - xMean).^2);

% finding the term of sum of difference of y axis - yfit squared
SSres = sum((liftForce - windForcey).^2);

% finding Syx
Syx = sqrt(SSres/dF);

% fit confidence interval max and min
fitCf = t*Syx*sqrt(1/N + ((windSpeed - xMean).^2)/SSxx);
minFitCf = windForcey - fitCf;
maxFitCf = windForcey + fitCf;

% measurement confidence interval max and min
measCf = t*Syx*sqrt(1 + 1/N + ((windSpeed - xMean).^2)/SSxx);
minMeaCf = windForcey - measCf;
maxMeaCf = windForcey + measCf;
% confidence interval ends--------------------------------------

%% Plotting everythang
figure;
plot(windSpeed,liftForce,'x',windForcex,windForcey);
hold on;
p1 = plot(windSpeed,minFitCf,'r:');
p2 = plot(windSpeed,maxFitCf,'r:');
p3 = plot(windSpeed,minMeaCf,'b--');
p4 = plot(windSpeed,maxMeaCf,'b--');
set(get(get(p2,'Annotation'),'LegendInformation'),...
    'IconDisplayStyle','off');
set(get(get(p4,'Annotation'),'LegendInformation'),...
    'IconDisplayStyle','off');
xlabel('Wind Speed (m/s)','FontSize',12);
ylabel('Lift Force (N)','FontSize',12);
title('Lift Force vs. Wind Speed for Stella Cylinder','FontSize',14);
l = legend('Location','best','Experiment Data','Best fit line','Confidence of Fit','Confidence of Measurement');
set(l,'FontSize',12);

%% Calculateing the Coefficient of Lift 
%  THEORETICAL

% Below calculations were done using the equation found 
% in Tokumaru pdf
% Note: in the document a = radius
% CL = Lift/rhoair*(velocity.^2)*radius

% Coeffeicient of lift for Stella
CLthStella1 = FLStella1/airrho*(speed1^2)*rStella;
CLthStella2 = FLStella2/airrho*(speed2^2)*rStella;
CLthStella3 = FLStella3/airrho*(speed3^2)*rStella;
CLthStella4 = FLStella4/airrho*(speed4^2)*rStella;

% Coeffeicient of lift for Bud Heavy
CLthBud1 = FLBud1/airrho*(speed1^2)*rBud;
CLthBud2 = FLBud2/airrho*(speed2^2)*rBud;
CLthBud3 = FLBud3/airrho*(speed3^2)*rBud;
CLthBud4 = FLBud4/airrho*(speed4^2)*rBud;

% Coeffeicient of lift for Oats
CLthQuaker1 = FLQuaker1/airrho*(speed1^2)*rQuaker;
CLthQuaker2 = FLQuaker2/airrho*(speed2^2)*rQuaker;
CLthQuaker3 = FLQuaker3/airrho*(speed3^2)*rQuaker;
CLthQuaker4 = FLQuaker4/airrho*(speed4^2)*rQuaker;

%% Plot Theoretical Coefficent of Lift

figure
hold on
plot(RPMMagnusStella,CLthStella1,'-')
plot(RPMMagnusStella,CLthStella2,'-')
plot(RPMMagnusStella,CLthStella3,'-')
plot(RPMMagnusStella,CLthStella4,'-')
title('Theoretical Coefficient of Lift for Stella Cylinder')
xlabel('Velocity (m/s)')
ylabel('CL')
legend('Location','northwest','12 m/s','16 m/s','20 m/s','24 m/s')

figure
hold on
plot(RPMMagnusBud,CLthBud1,'-')
plot(RPMMagnusBud,CLthBud2,'-')
plot(RPMMagnusBud,CLthBud3,'-')
plot(RPMMagnusBud,CLthBud4,'-')
title('Theoretical Coefficient of Lift for Bud Heavy Cylinder')
xlabel('Velocity (m/s)')
ylabel('CL')
legend('Location','northwest','12 m/s','16 m/s','20 m/s','24 m/s')

figure
hold on
plot(RPMMagnusQuaker,CLthQuaker1,'-')
plot(RPMMagnusQuaker,CLthQuaker2,'-')
plot(RPMMagnusQuaker,CLthQuaker3,'-')
plot(RPMMagnusQuaker,CLthQuaker4,'-')
title('Theoretical Coefficient of Lift for Quaker Oat Cylinder')
xlabel('Velocity (m/s)')
ylabel('CL')
legend('Location','northwest','12 m/s','16 m/s','20 m/s','24 m/s')

%% Calculateing the Coefficient of Lift 
%  EXPERIMENTAL

% Below calculations were done using the equation found 
% in Tokumaru pdf
% Note: in the document a = radius
% CL = Lift/rhoair*(velocity.^2)*radius

% All speeds in order of smallest to greatest magnitude of velocity
%speedslr = fliplr(speeds);

% Coeffeicient of lift for Stella
% Organized by trial (Constant windpeed, changing rpm)
CLexpStella1 = forceStella(1,:)./airrho*(speed1^2)*rStella;
CLexpStella2 = forceStella(2,:)./airrho*(speed2^2)*rStella;
CLexpStella3 = forceStella(3,:)./airrho*(speed3^2)*rStella;
CLexpStella4 = forceStella(4,:)./airrho*(speed4^2)*rStella;

% Coeffeicient of lift for Bud Heavy
% Organized by trial (Constant windpeed, changing rpm)
CLexpBud1 = forceBud(1,:)./airrho*(speed1^2)*rBud;
CLexpBud3 = forceBud(3,:)./airrho*(speed3^2)*rBud;
CLexpBud4 = forceBud(4,:)./airrho*(speed4^2)*rBud;


% Coeffeicient of lift for Oats
% Organized by trial (Constant windpeed, changing rpm)
CLexpQuaker1 = forceQuaker(1,:)./airrho*(speed1^2)*rQuaker;
CLexpQuaker2 = forceQuaker(2,:)./airrho*(speed2^2)*rQuaker;
CLexpQuaker3 = forceQuaker(3,:)./airrho*(speed3^2)*rQuaker;
CLexpQuaker4 = forceQuaker(4,:)./airrho*(speed4^2)*rQuaker;

%% Plot for Experimental Coefficient of Lift

figure
hold on
plot(RPMStella(1,:),CLexpStella1,'--o')
plot(RPMStella(2,:),CLexpStella2,'--o')
plot(RPMStella(3,:),CLexpStella3,'--o')
plot(RPMStella(4,:),CLexpStella4,'--o')
title('Experimental Coefficient of Lift for Stella Cylinder')
xlabel('Velocity (m/s)')
ylabel('CL')
legend('Location','northwest','12 m/s','16 m/s','20 m/s','24 m/s')

figure
hold on
plot(RPMBud(1,:),CLexpBud1,'--o')
plot(RPMBud(3,:),CLexpBud3,'--o')
plot(RPMBud(4,:),CLexpBud4,'--o')
title('Experimental Coefficient of Lift for Bud Heavy Cylinder')
xlabel('Velocity (m/s)')
ylabel('CL')
legend('Location','northwest','12 m/s','20 m/s','24 m/s')

figure
hold on
plot(RPMQuaker(1,:),CLexpQuaker1,'--o')
plot(RPMQuaker(2,:),CLexpQuaker2,'--o')
plot(RPMQuaker(3,:),CLexpQuaker3,'--o')
plot(RPMQuaker(4,:),CLexpQuaker4,'--o')
title('Experimental Coefficient of Lift for Quaker Oat Cylinder')
xlabel('Velocity (m/s)')
ylabel('CL')
legend('Location','northwest','12 m/s','16 m/s','20 m/s','24 m/s')
