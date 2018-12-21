function R = dad(s, t)
% Lie bracket algebra between dse(3) elements
% R = dad(s, t)
% s : 6*1, t : 6*1, R : 6*1
% LieGroup Matlab Library [Syungkwon Ra, dearLenin@gmail.com]

if (size(s) == [6,1] & size(t) == [6,1])
    R = [   t(2) * s(3) - t(3) * s(2) + t(5) * s(6) - t(6) * s(5);
			t(3) * s(1) - t(1) * s(3) + t(6) * s(4) - t(4) * s(6);
			t(1) * s(2) - t(2) * s(1) + t(4) * s(5) - t(5) * s(4);
			t(5) * s(3) - t(6) * s(2);
			t(6) * s(1) - t(4) * s(3);
			t(4) * s(2) - t(5) * s(1) ];
    return;
else error('Elements must have size of 6 row and 1 column');
end;