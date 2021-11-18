function SE3_MAT = TRANSX(x)
% Return SE3 rotational matrix based on x translation value
%   input : x value to translate
%   output : SE3_MAT
%   SE3_MAT = SE3 matrix based on input x
    SE3_MAT = [1 0 0 x;
               0 1 0 0;
               0 0 1 0;
               0 0 0 1];
end