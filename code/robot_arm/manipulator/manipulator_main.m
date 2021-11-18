addpath("lib");
clc;
clear;
close;

l1 = 0.077;
l21 = 0.128;
l22 = 0.024;
l3 = 0.124;
l4 = 0.126;

q10 = 0;
q20 = pi;
q30 = pi;
q40 = pi/2;

syms q1 q2 q3 q4 real

% world to base motor(1)
R01 = ROTZ(q1);
p01 = zeros(3,1);
T01 = [R01 p01;
    0 0 0 1];

% base motor(1) to motor 2
% ROTY90 = ROTY(pi/2)
ROTY90 = [0 0 1;
        0 1 0;
        -1 0 0];
R12 = ROTY90*ROTZ(q2);
p12 = [0; 0; l1];
T12 = [R12 p12;
    0 0 0 1];

% motor 2 to motor 3
R23 = ROTZ(q3);
p23 = [-l21; -l22; 0];
T23 = [R23 p23;
    0 0 0 1];

% motor 3 to motor 4
R34 = ROTZ(q4);
p34 = [0; -l3; 0];
T34 = [R34 p34;
    0 0 0 1];

% motor 4 to end effector
R45 = eye(3);
p45 = [l4; 0; 0];
T45 = [R45 p45;
    0 0 0 1];

WORLD = [0;0;0;1];
BASE = T01 * WORLD
MOTOR2 = (T01 * T12) * WORLD
MOTOR3 = (T01 * T12 * T23) * WORLD
MOTOR4 = (T01 * T12 * T23 * T34) * WORLD
END = (T01 * T12 * T23 * T34 * T45) * WORLD

%% visualize

q1 = 0.454363;
q2 = -0.247039;
q3 = 0.263162;
q4 = 0;

% world to base motor(1)
R01 = ROTZ(q1);
p01 = zeros(3,1);
T01 = [R01 p01;
    0 0 0 1];

% base motor(1) to motor 2
% ROTY90 = ROTY(pi/2)
ROTY90 = [0 0 1;
        0 1 0;
        -1 0 0];
R12 = ROTY90*ROTZ(q2);
p12 = [0; 0; l1];
T12 = [R12 p12;
    0 0 0 1];

% motor 2 to motor 3
R23 = ROTZ(q3);
p23 = [-l21; -l22; 0];
T23 = [R23 p23;
    0 0 0 1];

% motor 3 to motor 4
R34 = ROTZ(q4);
p34 = [0; -l3; 0];
T34 = [R34 p34;
    0 0 0 1];

% motor 4 to end effector
R45 = eye(3);
p45 = [l4; 0; 0];
T45 = [R45 p45;
    0 0 0 1];

WORLD = [0;0;0;1];
BASE = T01 * WORLD;
MOTOR2 = (T01 * T12) * WORLD;
MOTOR3 = (T01 * T12 * T23) * WORLD;
MOTOR4 = (T01 * T12 * T23 * T34) * WORLD;
END = (T01 * T12 * T23 * T34 * T45) * WORLD;

view([30 -20 3]);

loc1 = BASE; loc2 = MOTOR2;
k1=line([loc1(1) loc2(1)],[loc1(2) loc2(2)],[loc1(3) loc2(3)],'Linewidth',2,'Color','r'); hold on;

loc1 = MOTOR2; loc2 = MOTOR3;
k2=line([loc1(1) loc2(1)],[loc1(2) loc2(2)],[loc1(3) loc2(3)],'Linewidth',2,'Color','g'); hold on;

loc1 = MOTOR3; loc2 = MOTOR4;
k3=line([loc1(1) loc2(1)],[loc1(2) loc2(2)],[loc1(3) loc2(3)],'Linewidth',2,'Color','b'); hold on;

loc1 = MOTOR4; loc2 = END;
k4=line([loc1(1) loc2(1)],[loc1(2) loc2(2)],[loc1(3) loc2(3)],'Linewidth',2,'Color','k'); hold on;

END

grid on
xlabel('x'); ylabel('y'); zlabel('z');