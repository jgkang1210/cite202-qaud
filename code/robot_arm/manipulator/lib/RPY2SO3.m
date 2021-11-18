function SO3_MAT = RPY2SO3(r, p, y)
% Return SO3 rotational matrix based on roll, pitch, yaw value
%   input : roll, pitch yaw
%   r = roll value of frame
%   p = pitch value of frame
%   y = yaw value of frame
%   output : SO3_MAT
%   SO3_MAT = SO3 matrix based on input r, p, y
    SO3_MAT = [cos(p)*cos(y), sin(r)*sin(p)*cos(y) - cos(r)*sin(y), sin(r)*sin(y) + cos(r)*sin(p)*cos(y);
       cos(p)*sin(y), cos(r)*cos(y) + sin(r)*sin(p)*sin(y), cos(r)*sin(p)*sin(y) - sin(r)*cos(y);
       -sin(p), sin(r)*cos(p), cos(r)*cos(p)];
end