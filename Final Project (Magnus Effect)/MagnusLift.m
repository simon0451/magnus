RPM = 1000; %RPM of the airfoil after gearing
r = 0:.001:.16; %radius of the airfoil 
omega = RPM*0.10471975511966; %converting to radians per second
G = omega*2*pi.*r.^2; %calculating the vortex strength, G
v = 25; %meters per second, the velocity of the air in the wind tunnel
rho = 1.225; %hg/m^3 - density of air
L = .25; %length of the airfoil in meters
Force = rho*v.*G*L; %lift force

plot(r,Force);
xlabel('Airfoil Radius (m)');
ylabel('Lift Force (N)');
ylim([-10 110]);
xlim([-0.01 0.16]);
grid on;