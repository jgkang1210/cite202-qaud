function tau = controller(t,theta1,omega1,theta2,omega2,parms)
l1 = parms.l1;
l2 = parms.l2;
gamma = 0.001;

Jr11 = 0;
Jr12 = -(l1*l2*sin(theta2))/(l1^2 + l2^2 + 2*l1*l2*cos(theta2))^(1/2);
Jt11 = 1;
Jt12 = (l2*(l2 + l1*cos(theta2)))/(l1^2 + l2^2 + 2*l1*l2*cos(theta2));
J_r = [Jr11 Jr12];
J_t = [Jt11 Jt12];

J = [J_r; J_t];

% damped Jacobian
J = J + gamma * eye(2);

r = (l1^2 + l2^2 + 2*l1*l2*cos(theta2))^(1/2);
theta = pi/2 + atan2(l2*sin((3*pi)/2 + theta1 + theta2) + l1*sin((3*pi)/2 + theta1), l2*cos((3*pi)/2 + theta1 + theta2) + l1*cos((3*pi)/2 + theta1));

qdot = J*[omega1; omega2];
rdot = qdot(1);
thetadot = qdot(2);

% parms.control.dr = 1.5; parms.control.dt = 0; % desired r and theta
% parms.control.drd = 0; parms.control.dtd = 0; % desired r dot and theta dot
% parms.control.kpr = 20; parms.control.kpt = 1; % stiffness
% parms.control.kdr = 10; parms.control.kdt = 0; % damping
% parms.control = 0; % 0 for off, 1 for on

amplitude = [0, 0]';
frequency = [0, 0]'*2*pi;
offset = [parms.control.dr, parms.control.dt]';
q_b_des = amplitude.*sin(frequency*t) + offset;
qd_b_des = amplitude.*frequency.*cos(frequency*t);


F_r = parms.control.kpr*(q_b_des(1)-r) + ...
    parms.control.kdr*(q_b_des(2)-rdot); % force in radial direction
F_t = parms.control.kpt*(qd_b_des(1)-theta) + ...
    parms.control.kdt*(qd_b_des(2)-thetadot); % force in theta direction

tau = J'*[F_r; F_t];
end