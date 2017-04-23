%% Using the Stella Can as an Airfoil
RPM = 60:1:2000; %RPM of the airfoil after gearing (bronze bearings limit this value to 5,725 RPM)
rStella = .02794; %radius of the airfoil 
omega = RPM*0.10471975511966; %converting to radians per second
G = omega*2*pi.*rStella.^2; %calculating the vortex strength, G
v = 20; %meters per second, the velocity of the air in the wind tunnel
rho = 1.2; %hg/m^3 - density of air
L = .127; %length of the airfoil in meters
Force = rho*v.*G*L; %lift force

figure(1)
plot(RPM,Force);
title('Stella Can Airfoil, 20 m/s Airspeed')
xlabel('Airfoil Rotational Speed (RPM)');
ylabel('Lift Force (N)');
grid on;

%% Using the Bud Heavy Can as an Airfoil
RPM = 60:1:2000; %RPM of the airfoil after gearing (bronze bearings limit this value to 5,725 RPM)
rBud = .0418; %radius of the airfoil 
omega = RPM*0.10471975511966; %converting to radians per second
G = omega*2*pi.*rBud.^2; %calculating the vortex strength, G
v = 20; %meters per second, the velocity of the air in the wind tunnel
rho = 1.2; %hg/m^3 - density of air
L = .157; %length of the airfoil in meters
Force = rho*v.*G*L; %lift force

figure(2)
plot(RPM,Force);
title('Bud Heavy Can Airfoil, 20 m/s Airspeed')
xlabel('Airfoil Rotational Speed (RPM)');
ylabel('Lift Force (N)');
grid on;

%% Using the Quaker Oats Can as an Airfoil
RPM = 60:1:2000; %RPM of the airfoil after gearing (bronze bearings limit this value to 5,725 RPM)
rQ = .0635; %radius of the airfoil 
omega = RPM*0.10471975511966; %converting to radians per second
G = omega*2*pi.*rQ.^2; %calculating the vortex strength, G
v = 20; %meters per second, the velocity of the air in the wind tunnel
rho = 1.2; %hg/m^3 - density of air
L = .2286; %length of the airfoil in meters
Force = rho*v.*G*L; %lift force

figure(3)
plot(RPM,Force);
title('Quaker Oats Can Airfoil, 20 m/s Airspeed')
xlabel('Airfoil Rotational Speed (RPM)');
ylabel('Lift Force (N)');
grid on;




