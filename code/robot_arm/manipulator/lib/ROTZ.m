function SO3_MAT = ROTZ(theta)
% Return SO3 rotational matrix based on theta value
%   input : theta
%   theta = angle to rotate about inertial z axis
%   output : SO3_MAT
%   SO3_MAT = SO3 matrix based on input theta
    SO3_MAT = [cos(theta) -sin(theta) 0;
               sin(theta) cos(theta) 0;
               0 0 1;];
end