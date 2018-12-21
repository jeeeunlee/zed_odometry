function output = se3(input)
% se(3) matrix form converting from 6*1 to 4*4
% or convert from 6*1 to 4*4

if (size(input) == [6,1])
    % case of conveting 6*1 to 4*4
    s = input;
    output = [    0    -s(3)  s(2)  s(4);
             s(3)  0    -s(1)  s(5);
            -s(2)  s(1)  0     s(6);
             0     0     0     0    ];
    return;
elseif (size(input) == [4,4])
    % case of conveting 4*4 to 6*1
    S = input;
    output = [S(2,3); S(3,1); S(2,1); S(1:3,4);];
    return;
else error('Elements must have size of 6 row and 1 column');
end;

