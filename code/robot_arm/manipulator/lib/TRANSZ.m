function SE3_MAT = TRANSZ(z)
% Return SE3 rotational matrix based on z translation value
%   input : x value to translate
%   output : SE3_MAT
%   SE3_MAT = SE3 matrix based on input z
    SE3_MAT = [1 0 0 0;
               0 1 0 0;
               0 0 1 z;
               0 0 0 1];
end