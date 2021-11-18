function SE3_MAT = TRANSY(y)
% Return SE3 rotational matrix based on y translation value
%   input : x value to translate
%   output : SE3_MAT
%   SE3_MAT = SE3 matrix based on input y
    SE3_MAT = [1 0 0 0;
               0 1 0 y;
               0 0 1 0;
               0 0 0 1];
end