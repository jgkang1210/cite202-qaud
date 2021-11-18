function SO3_MAT = ROTX(theta)
% Return SO3 rotational matrix based on theta value
%   input : theta
%   theta = angle to rotate about inertial x axis
%   output : SO3_MAT
%   SO3_MAT = SO3 matrix based on input theta
    SO3_MAT = [1 0 0;
               0 cos(theta) -sin(theta);
               0 sin(theta) cos(theta);];
end