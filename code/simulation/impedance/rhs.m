function zdot = rhs(t,z,parms)

m1 = parms.m1; m2 = parms.m2;
I1 = parms.I1; I2 = parms.I2;
c1 = parms.c1; c2 = parms.c2;
l1 = parms.l1; l2 = parms.l2; g = parms.g;

theta1 = z(1);
omega1 = z(2);
theta2 = z(3);
omega2 = z(4);

M11 = I1 + I2 + (m2*(2*(c2*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)) + l1*cos(theta1))^2 + 2*(c2*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1)) + l1*sin(theta1))^2))/2 + (m1*(2*c1^2*cos(theta1)^2 + 2*c1^2*sin(theta1)^2))/2;
M12 = I2 + (m2*(2*c2*(c2*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)) + l1*cos(theta1))*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)) + 2*c2*(c2*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1)) + l1*sin(theta1))*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1))))/2;
M21 = I2 + (m2*(2*c2*(c2*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)) + l1*cos(theta1))*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)) + 2*c2*(c2*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1)) + l1*sin(theta1))*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1))))/2;
M22 = I2 + (m2*(2*c2^2*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1))^2 + 2*c2^2*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2))^2))/2;
C1 = -(m2*omega2*(2*(c2*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)) + l1*cos(theta1))*(c2*omega1*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1)) + c2*omega2*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1))) - 2*(c2*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1)) + l1*sin(theta1))*(c2*omega1*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)) + c2*omega2*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2))) + 2*c2*(omega1*(c2*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)) + l1*cos(theta1)) + c2*omega2*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)))*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1)) - 2*c2*(omega1*(c2*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1)) + l1*sin(theta1)) + c2*omega2*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1)))*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2))))/2;
C2 = (m2*(2*(omega1*(c2*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)) + l1*cos(theta1)) + c2*omega2*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)))*(c2*omega1*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1)) + c2*omega2*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1))) - 2*(omega1*(c2*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1)) + l1*sin(theta1)) + c2*omega2*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1)))*(c2*omega1*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)) + c2*omega2*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)))))/2 - (m2*omega2*(2*c2*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2))*(c2*omega1*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1)) + c2*omega2*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1))) - 2*c2*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1))*(c2*omega1*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)) + c2*omega2*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2))) + 2*c2*(omega1*(c2*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)) + l1*cos(theta1)) + c2*omega2*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)))*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1)) - 2*c2*(omega1*(c2*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1)) + l1*sin(theta1)) + c2*omega2*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1)))*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2))))/2;
G1 = g*m2*(c2*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1)) + l1*sin(theta1)) + c1*g*m1*sin(theta1);
G2 = c2*g*m2*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1));
M = [M11 M12; M21 M22];
C = [C1; C2];
G = [G1; G2];
thetaddot = M\(-G-C);

t

% turn on the controller
if parms.control.on == 1
    tau = controller(t,theta1,omega1,theta2,omega2,parms);
    %tau = tau + G + C; % gravity and coriolis force compensation
else
    tau_1 = 0; %shoulder torque
    tau_2 = 0; %-0.5*omega2; %-100*(theta2-pi/2); % 1, 10, 100 %elbow torque
    tau = [tau_1; tau_2];
end

thetaddot =M\(tau-G-C); % M\(tau-G-C);

zdot = [omega1 thetaddot(1) omega2 thetaddot(2)]';
end