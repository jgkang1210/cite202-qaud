syms l1 l2 theta1 theta2 theta1d theta2d real

q = [theta1 theta2];

% axis is rotated by 3*pi/2
x = l1*cos(theta1+3*pi/2) + l2*cos(theta1+theta2+3*pi/2);
y = l1*sin(theta1+3*pi/2) + l2*sin(theta1+theta2+3*pi/2);

r = simplify(sqrt(x^2+y^2));
theta = atan2(y,x)+pi/2;

J_r = simplify(jacobian(r,q)) % jacobian in r direction
J_t = simplify(jacobian(theta,q)) % jacobian in theta direction

disp('copy paste in MATLAB');
disp(' ');

disp(['r = ', char(r), ';']);
disp(['theta = ', char(theta), ';']);

disp(['Jr11 = ', char(J_r(1,1)), ';']);
disp(['Jr12 = ', char(J_r(1,2)), ';']);
disp(['Jt11 = ', char(J_t(1,1)), ';']);
disp(['Jt12 = ', char(J_t(1,2)), ';']);

disp(['J_r = [Jr11 Jr12];']);
disp(['J_t = [Jt11 Jt12];']);

l1 = 1;
l2 = 1;
theta1 = pi/2;
theta2 = pi/2;
x = l1*cos(theta1+3*pi/2) + l2*cos(theta1+theta2+3*pi/2);
y = l1*sin(theta1+3*pi/2) + l2*sin(theta1+theta2+3*pi/2);
r = (l1^2 + l2^2 + 2*l1*l2*cos(theta2))^(1/2);
theta = pi/2 + atan2(l2*sin((3*pi)/2 + theta1 + theta2) + l1*sin((3*pi)/2 + theta1), l2*cos((3*pi)/2 + theta1 + theta2) + l1*cos((3*pi)/2 + theta1));
Jr11 = 0;
Jr12 = -(l1*l2*sin(theta2))/(l1^2 + l2^2 + 2*l1*l2*cos(theta2))^(1/2);
Jt11 = 1;
Jt12 = (l2*(l2 + l1*cos(theta2)))/(l1^2 + l2^2 + 2*l1*l2*cos(theta2));
J_r = [Jr11 Jr12];
J_t = [Jt11 Jt12];

disp("results")
disp([x,y,r,rad2deg(theta)]')
