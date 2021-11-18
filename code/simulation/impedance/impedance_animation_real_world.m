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
parms.control.dr = 0.4; parms.control.dt = 0; % desired r and theta
parms.control.drd = 0; parms.control.dtd = 0; % desired r dot and theta dot
parms.control.kpr = 1; parms.control.kpt = 1; % stiffness
parms.control.kdr = 0;
parms.control.kdt = 0; % damping
parms.control.on = 1; % 0 for off, 1 for on

t = 0:0.005:8; %time values
z0 = [0.14 0 0 0];
options = odeset('Abstol',1e-6,'Reltol',1e-6);
[t, z] = ode45(@rhs,t,z0,options,parms); %integrate using ode45

file1 = 'dosc_r_0.4_t_0_1-1-01-01.txt';
file2 = 'osc_r_0.4_t_0_1100.txt';
%M = csvread(file1);
M = csvread(file2);

treal = 0:0.017345:27.752;

Mreal = zeros(length(treal),4);
Mreal(:,1) = M(1:length(treal),1);
Mreal(:,3) = M(1:length(treal),2);

video.fps = 50;
video.pauseTime = 0.01;
video.write = 1;
video.name = './video/simulate_osc';

figure(1)
animate(t,z,parms,video);

% 
% figure(1)
% subplot(2,1,1)
% plot(t,z(:,1),'r--','Linewidth',3); hold on
% plot(t,z(:,3),'b','Linewidth',2);
% ylabel('position','Fontsize',12);
% title('Leg position and velocity as a function of time','Fontsize',12);
% legend('\theta_1','\theta_2','Location','best','Fontsize',12);
% subplot(2,1,2)
% plot(t,z(:,2),'r--','Linewidth',3); hold on
% plot(t,z(:,4),'b','Linewidth',2);
% ylabel('velocity','Fontsize',12);
% xlabel('time','Fontsize',12);
% legend('\theta_1','\theta_2','Location','best','Fontsize',12);
% 
figure(2)
treal = treal(865:1327);
treal = treal - 15;
randthetaval = randtheta(l1,l2,Mreal);
randthetaval = randthetaval(865:1327,:);

randthetaval2 = randtheta(l1,l2,z);

subplot(2,1,1)
plot(treal,randthetaval(:,1),'r','Linewidth',3); hold on
plot(t,randthetaval2(:,1),'b--','Linewidth',1.5); hold on
ylabel('radial','Fontsize',12);
title('Leg radial length and angle as a function of time','Fontsize',12);
legend('radial real','radial sim','Location','best','Fontsize',12);
subplot(2,1,2)
plot(treal,randthetaval(:,2),'r','Linewidth',3); hold on
plot(t,randthetaval2(:,2),'b--','Linewidth',1.5); hold on
ylabel('theta','Fontsize',12);
xlabel('time','Fontsize',12);
legend('\theta real','\theta sim','Location','best','Fontsize',12);

% 
% figure(3)
% animate(t,z,parms,video);