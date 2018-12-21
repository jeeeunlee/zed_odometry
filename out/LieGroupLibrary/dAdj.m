function R = dAdj(T, s)
% Dual adjoint mapping of SE(3) T and dse(3) s
% Dual adjoint mapping of vec3 T and dse(3) s (T's orientation is I)
% R = dAdj(T, s)
% T : 4*4 or 3*1, s : 6*1, R = 6*1
% LieGroup Matlab Library [Syungkwon Ra, dearLenin@gmail.com]

if (size(T) == [4,4] & size(s) == [6,1])
    tmp = [ s(1) - T(2,4) * s(6) + T(3,4) * s(5); 
			s(2) - T(3,4) * s(4) + T(1,4) * s(6); 
			s(3) - T(1,4) * s(5) + T(2,4) * s(4) ];
	R = [   T(1,1) * tmp(1) + T(2,1) * tmp(2) + T(3,1) * tmp(3);
			T(1,2) * tmp(1) + T(2,2) * tmp(2) + T(3,2) * tmp(3);
			T(1,3) * tmp(1) + T(2,3) * tmp(2) + T(3,3) * tmp(3);
			T(1,1) * s(4) + T(2,1) * s(5) + T(3,1) * s(6);
			T(1,2) * s(4) + T(2,2) * s(5) + T(3,2) * s(6);
			T(1,3) * s(4) + T(2,3) * s(5) + T(3,3) * s(6) ];
    return;
elseif (size(T) == [3,1] & size(S) == [6,1])
	R = [   s(1) - T(2,4) * s(6) + T(3,4) * s(5);
		    s(2) - T(3,4) * s(4) + T(1,4) * s(6);
			s(3) - T(1,4) * s(5) + T(2,4) * s(4);
			s(4);
			s(5);
			s(6) ];
    return;
else error('Dual Adjoint Mapping needs first element for SE(3) or Vec(3) and second element for dse(3)');
end;