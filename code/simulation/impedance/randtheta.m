function randtheta = randtheta(l1,l2,z)
theta1 = z(:,1);
theta2 = z(:,3);

r = sqrt((l1^2 + l2^2 + 2*l1*l2*cos(theta2)));
theta = pi/2 + atan2(l2*sin((3*pi)/2 + theta1 + theta2) + l1*sin((3*pi)/2 + theta1), l2*cos((3*pi)/2 + theta1 + theta2) + l1*cos((3*pi)/2 + theta1));

randtheta = [r theta];

end