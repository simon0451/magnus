%% Checking a range of values for radius and rotation speed
RPM = linspace(60,2000,100); %RPM of the airfoil after gearing (bronze bearings limit this value to 5,725 RPM)
rStella = linspace(0.02,0.065,100); %radius [m] of the airfoil
omega = RPM*0.10471975511966; %converting to radians per second
G = omega.*2*pi.*rStella.^2; %calculating the vortex strength, G
v = 20; %meters per second, the velocity of the air in the wind tunnel
rho = 1.2; %hg/m^3 - density of air
L = .127; %length of the airfoil in meters
[rspeed,radius] = meshgrid(RPM,rStella);
linForce = rho*v*G*L; %lift force linear to show individual relationship

% meshed lift force to show combined relationship
Force = zeros(100,100);
for i = 1:100
    for j = 1:100
        Force(i,j) = rho*v*omega(i)*2*pi*rStella(j)^2*L; %lift force
    end
end

figure;
surf(rspeed,radius,Force);
title('Magnus Lift on Cylindrical Airfoil, 20 m/s Airspeed')
xlabel('Airfoil Rotational Speed (RPM)');
ylabel('Cylinder Radius (m)');
zlabel('Lift Force (N)');
shading interp

figure;
plot(RPM,linForce);
title('Magnus Lift on Cylindrical Airfoil, 20 m/s Airspeed')
xlabel('Airfoil Rotational Speed (RPM)');
ylabel('Lift Force (N)');
grid on;

figure;
plot(rStella,linForce);
title('Magnus Lift on Cylindrical Airfoil, 20 m/s Airspeed')
xlabel('Cylinder Radius (m)');
ylabel('Lift Force (N)');
grid on;