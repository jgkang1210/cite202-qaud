function SE3_MAT = TRANS_SE3(x,y,z)
% Return SE3 rotational matrix based on x,y,z translation value
%   input : x, y, z value to translate
%   output : SE3_MAT
%   SE3_MAT = SE3 matrix based on input x,y,z
    SE3_MAT = [1 0 0 x;
               0 1 0 y;
               0 0 1 z;
               0 0 0 1];
end