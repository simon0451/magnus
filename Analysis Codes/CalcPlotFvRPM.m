function CalcPlotFvRPM(r,sMagnus,sExp,forceExp,L)
% function will compute and plot the expected magnus effect and the
% experimentally measured values

speed4 = 24.01946618; % [m/s]
speed3 = 19.78614266;
speed2 = 16.36650742;
speed1 = 12.00973309;
airrho = 1.2093; % from ideal gas law and the barometric pressure of the day

% FL = 4*pi^2*r^2*rho*v*omega*L
FLStella4 = 4*pi^2*r^2*airrho*speed4.*sMagnus*L;
FLStella3 = 4*pi^2*r^2*airrho*speed3.*sMagnus*L;
FLStella2 = 4*pi^2*r^2*airrho*speed2.*sMagnus*L;
FLStella1 = 4*pi^2*r^2*airrho*speed1.*sMagnus*L;

figure;
% speed4 plots
subplot(2,2,1);
plot(sExp(1,:)*60,forceExp(1,:),'bx',sMagnus*60,FLStella4,'r--');
xlabel('Cylinder Rotation Speed [RPM]');
ylabel('Lift Force (N)');
title('Lift Force vs. RPM at 23.9 m/s');
legend('Experiment Data','Magnus Predicted','Location','best');

% speed3 plots
subplot(2,2,2);
plot(sExp(2,:)*60,forceExp(2,:),'bx',sMagnus*60,FLStella3,'r--');
xlabel('Cylinder Rotation Speed [RPM]');
ylabel('Lift Force (N)');
title('Lift Force vs. RPM at 19.7 m/s');
legend('Experiment Data','Magnus Predicted','Location','best');

% speed2 plots
subplot(2,2,3);
plot(sExp(3,:)*60,forceExp(3,:),'bx',sMagnus*60,FLStella2,'r--');
xlabel('Cylinder Rotation Speed [RPM]');
ylabel('Lift Force (N)');
title('Lift Force vs. RPM at 16.3 m/s');
legend('Experiment Data','Magnus Predicted','Location','best');

% speed1 plots
subplot(2,2,4);
plot(sExp(4,:)*60,forceExp(4,:),'bx',sMagnus*60,FLStella1,'r--');
xlabel('Cylinder Rotation Speed [RPM]');
ylabel('Lift Force (N)');
title('Lift Force vs. RPM at 11.9 m/s');
legend('Experiment Data','Magnus Predicted','Location','best');

end