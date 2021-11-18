clc
clear all
close all

m1 = 0.570; I1 = 3749272*1e-9; m2 = 0.672; I2 = 3462715*1e-9;
c1 = 0.08; c2 = 0.08;
l1 = 0.2; l2 = 0.21; g = 9.81;

parms.m1 = m1; parms.m2 = m2; 
parms.I1 = I1; parms.I2 = I2;
parms.c1 = c1; parms.c2 = c2;
parms.l1 = l1; parms.l2 = l2; parms.g = g;
parms.control.dr = 0.2; parms.control.dt = 0; % desired r and theta
parms.control.drd = 0; parms.control.dtd = 0; % desired r dot and theta dot
parms.control.kpr = 2; parms.control.kpt = 2; % stiffness
parms.control.kdr = 2*sqrt(parms.control.kpr);
parms.control.kdt = 2*sqrt(parms.control.kpt); % damping
parms.control.on = 1; % 0 for off, 1 for on

t = linspace(0,3); %time values
z0 = [pi/2 0 0 0];
options = odeset('Abstol',1e-6,'Reltol',1e-6);
[t, z] = ode45(@rhs,t,z0,options,parms); %integrate using ode45

video.fps = 30;
video.pauseTime = 0.01;
video.write = 0;
video.name = './video/r0.2t60f1kp200kt2000';

figure(1)
subplot(2,1,1)
plot(t,z(:,1),'r--','Linewidth',3); hold on
plot(t,z(:,3),'b','Linewidth',2);
ylabel('position','Fontsize',12);
title('Leg position and velocity as a function of time','Fontsize',12);
legend('\theta_1','\theta_2','Location','best','Fontsize',12);
subplot(2,1,2)
plot(t,z(:,2),'r--','Linewidth',3); hold on
plot(t,z(:,4),'b','Linewidth',2);
ylabel('velocity','Fontsize',12);
xlabel('time','Fontsize',12);
legend('\theta_1','\theta_2','Location','best','Fontsize',12);

figure(2)
randthetaval = randtheta(l1,l2,z);
subplot(2,1,1)
plot(t,randthetaval(:,1),'r','Linewidth',3); hold on
ylabel('radial','Fontsize',12);
title('Leg radial length and angle as a function of time','Fontsize',12);
legend('radial','Location','best','Fontsize',12);
subplot(2,1,2)
plot(t,randthetaval(:,2),'b','Linewidth',3); hold on
ylabel('theta','Fontsize',12);
xlabel('time','Fontsize',12);
legend('\theta','Location','best','Fontsize',12);

figure(3)
animate(t,z,parms,video);