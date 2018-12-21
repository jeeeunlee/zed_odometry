function R = adm(s1)
% Lie bracket algebra between se(3) elements
% R = ad(s1, s2)
% s1 : 6*1, s2 : 6*1, R : 6*1
% LieGroup Matlab Library [Syungkwon Ra, dearLenin@gmail.com]

if (size(s1) == [6,1])
    temp = so3(s1(1:3));
    R = [   temp            zeros(3);
            so3(s1(4:6))    temp];
    return;
else error('Elements must have size of 6 row and 1 column');
end;