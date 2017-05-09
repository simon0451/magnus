%% Using the Stella Can as an Airfoil
RPM = 60:1:2000; %RPM of the airfoil after gearing (bronze bearings limit this value to 5,725 RPM)
rStella = .028956; %radius of the airfoil 
omega = RPM*0.10471975511966; %converting to radians per second
G = omega*2*pi.*rStella.^2; %calculating the vortex strength, G
v = 20; %meters per second, the velocity of the air in the wind tunnel
rho = 1.2; %hg/m^3 - density of air
L = .130175; %length of the airfoil in meters
Force = rho*v.*G*L; %lift force

figure
plot(RPM,Force);
title('Stella Can Airfoil, 20 m/s Airspeed')
xlabel('Airfoil Rotational Speed (RPM)');
ylabel('Lift Force (N)');
grid on;

%% Using the Bud Heavy Can as an Airfoil
RPM = 60:1:2000; %RPM of the airfoil after gearing (bronze bearings limit this value to 5,725 RPM)
rBud = .04191; %radius of the airfoil 
omega = RPM*0.10471975511966; %converting to radians per second
G = omega*2*pi.*rBud.^2; %calculating the vortex strength, G
v = 20; %meters per second, the velocity of the air in the wind tunnel
rho = 1.2; %hg/m^3 - density of air
L = .1762125; %length of the airfoil in meters
Force = rho*v.*G*L; %lift force

figure
plot(RPM,Force);
xlim([0 1000]);
ylim([0 5]);
title('Bud Heavy Can Airfoil, 20 m/s Airspeed')
xlabel('Airfoil Rotational Speed (RPM)');
ylabel('Lift Force (N)');
grid on;

%% Using the Quaker Oats Can as an Airfoil
RPM = 60:1:2000; %RPM of the airfoil after gearing (bronze bearings limit this value to 5,725 RPM)
rQ = .0634135; %radius of the airfoil 
omega = RPM*0.10471975511966; %converting to radians per second
G = omega*2*pi.*rQ.^2; %calculating the vortex strength, G
v = 20; %meters per second, the velocity of the air in the wind tunnel
rho = 1.2; %hg/m^3 - density of air
L = .2286; %length of the airfoil in meters
Force = rho*v.*G*L; %lift force

figure
plot(RPM,Force);
xlim([0 500]);
ylim([0 5]);
title('Quaker Oats Can Airfoil, 20 m/s Airspeed')
xlabel('Airfoil Rotational Speed (RPM)');
ylabel('Lift Force (N)');
grid on;
