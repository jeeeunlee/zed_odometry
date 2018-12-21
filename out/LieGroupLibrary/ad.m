function R = ad(s1, s2)
% Lie bracket algebra between se(3) elements
% R = ad(s1, s2)
% s1 : 6*1, s2 : 6*1, R : 6*1
% LieGroup Matlab Library [Syungkwon Ra, dearLenin@gmail.com]

if (size(s1) == [6,1] & size(s2) == [6,1])
    R = [   s1(2) * s2(3) - s1(3) * s2(2);
		    s1(3) * s2(1) - s1(1) * s2(3);
		    s1(1) * s2(2) - s1(2) * s2(1);
		    s1(2) * s2(6) - s1(3) * s2(5) - s2(2) * s1(6) + s2(3) * s1(5);
		    s1(3) * s2(4) - s1(1) * s2(6) - s2(3) * s1(4) + s2(1) * s1(6);
		    s1(1) * s2(5) - s1(2) * s2(4) - s2(1) * s1(5) + s2(2) * s1(4) ];
    return;
else error('Elements must have size of 6 row and 1 column');
end;