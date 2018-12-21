function R = so3sqr(v)
% so(3) matrix form converting from 3*1 to 3*3
% R = so3(v)
% v : 3*1, R : 3*3
% LieGroup Matlab Library [Syungkwon Ra, dearLenin@gmail.com]
temp = [ v(1)^2 v(2)^2 v(3)^2 ];
if (size(v) == [3,1])
    R = [    -temp(2)-temp(3)   v(1)*v(2)           v(1)*v(3);
             v(1)*v(2)          -temp(1)-temp(3)    v(2)*v(3);
             v(1)*v(3)          v(2)*v(3)           -temp(1)-temp(2)   ];
    return;
else error('Elements must have size of 3 row and 1 column');
end;
