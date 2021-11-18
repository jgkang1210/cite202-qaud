function [R,p] = SE3TORP(T)
% Return rotational matrix and position vector based on SE(3) matrix
%   input : SE3 matrix
%   SO3_MAT = SO3 matrix based on input SE3 matrix
%   p = traslation matrix based on input SE3 matrix
    R = T(1:3,1:3);
    p = T(1:3,4);
end