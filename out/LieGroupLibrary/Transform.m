function R = Transform(f, J)
% Homogeneous transformation of generalized inertia matrix, dAdj(f) * J * Adj(f)
% R = Transform(f, J)
% f : 4*4, J : 6*6, R : 6*6
% LieGroup Matlab Library [Syungkwon Ra, dearLenin@gmail.com]

if (size(J) == [6,6] & size(f) == [4,4])
    rot = f(1:3,1:3);
    b = [   0      -f(3,4)  f(2,4);
            f(3,4)  0      -f(1,4);
            -f(2,4) f(1,4)  0      ];
    temp = [rot    zeros(3,3);
            b*rot  rot        ];
    R = temp' * J * temp;
    return;
else error('Transform must have 6*6 Inertia matrix and 4*4 SE(3) matrix as elements')
end;