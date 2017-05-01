
% Plot streamlines and pressure coefficent of an unviscious,uncomprimibile,irrotational 
% flow around a cylinder  section (radius = 1) that spins around the z axis (coming out 
% of the xy plane).
% This result is achieved by superimposition of elementary solution of the potential 
% function PHI, where [Ux , Uy] = GRAD(PHI) which comprehend Uniform Stream,Doublet,Vortex.
% This case rappresents a good example of Magnus Effect,that is the reason why spinning 
% balls have an effected trajectory.And rappresent the basis of Kutta-Joukowski theory.
% 
%
% INPUT
% V_i  = Asymptotic Speed
% G    = Angular Speed (positive anti-clockwise)
%
% -----EXAMPLE------
% V_i = 20
% G = 50 
%
% Created by: Dario Isola 
% DATA : 24/03/2004
% Politecnico di Milano - Aerospatial Engeeniering Departement
close all

V_i = input(' Asymptotic speed V_0 [m/s] = ');
G = input(' Circulation Value G [rad/s] [Anti-clockwise] = ');
a = .25 ;   %RADIUS
c =-a*2;
b =a*2;
n =a*50; % NUMBER OF INTERVALS

[x,y]=meshgrid([c:(b-c)/n:b],[c:(b-c)/n:b]');

warning off
%Preliminar DATA & purification

for i=1:length(x);
   for k=1:length(x);
      if sqrt(x(i,k).^2+y(i,k).^2)<a;
         x(i,k)=0;
         y(i,k)=0;
      end
   end
end

%definition of polar variables
rho=sqrt(x.^2+y.^2);
theta=atan2(y,x);

% Creation of the streamline function
z=V_i.*sin(theta).*rho.*(1-(a^2./(rho.^2)))-G*log(rho)/(2*pi);
% Creation of the figure
n=100;
r=ones(1,n+1)*a;
t=[0:2*pi/n:2*pi];

%stream line Plot 
contour(x,y,z,25)
hold on
polar(t,r,'-k')
axis square
title('Stream Lines')
grid off
figure(2)
contour(x,y,z,15)

%SECOND PART
%recreation of X,Y 
%Creation of vectors around the circle
x=[-a*2:a/3:a*2];
[x]=meshgrid(x);
y=x';

for i=1:length(x);
   for k=1:length(x);
      if sqrt(x(i,k).^2+y(i,k).^2)<a;
         x(i,k)=0;
         y(i,k)=0;
      end
   end
end

r=sqrt(x.^2+y.^2);
theta=atan2(y,x);
ur=V_i*cos(theta).*(1-a^2./(r.^2));
ut=-V_i*sin(theta).*(1+a^2./(r.^2))+G./(2*pi*r);
u=ur.*cos(theta)-ut.*sin(theta);
v=ur.*sin(theta)+ut.*cos(theta);

%Creating The Filled Circle
t_r  = 0:.1:2*pi;
xxx = a*cos(t_r);
yyy = a*sin(t_r);



%Vectors and Filled Circle plotting
figure(2)
hold on
quiver(x,y,u,v)
fill(xxx,yyy,'y')
axis square
title('Speed Vectors')
grid off

warning on

t=0:.1:2*pi;
cp = 1 - 4*sin(t).^2 + 2* G / (pi*a*V_i) *sin(t) - (2* G/ (pi*a*V_i) )^2 ;
cp_sim = 1 - 4*sin(t).^2 ;
L = - 1.225*V_i*G;
L = num2str(L);
L = strcat('Kutta Joukowski Lift:  ',L,'  [N]');

figure(3)
plot(t,cp,t,cp_sim,'--r',0,0,'-w')
axis([0 2*pi min(cp) max(cp_sim)])
title('Pressure coefficient around the surface (standard air density)')
xlabel('Theta (angle with orizonthal)')
ylabel('C_p')
legend('Lifting solution','Symmetrical solution',L)
grid on


