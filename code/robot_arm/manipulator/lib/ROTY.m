function SO3_MAT = ROTY(theta)
% Return SO3 rotational matrix based on theta value
%   input : theta
%   theta = angle to rotate about inertial y axis
%   output : SO3_MAT
%   SO3_MAT = SO3 matrix based on input theta
    SO3_MAT = [cos(theta) 0 sin(theta);
               0 1 0;
               -sin(theta) 0 cos(theta);];
end