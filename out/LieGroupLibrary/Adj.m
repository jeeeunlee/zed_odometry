function R = Adj(T, s)
% Adjoint mapping of SE(3) T and se(3) s
% Adjoint mapping of vec3 T and se(3) s (T's orientation is I)
% R = Adj(T, s)
% T : 4*4 or 3*1, s : 6*1, R : 6*1
% LieGroup Matlab Library [Syungkwon Ra, dearLenin@gmail.com]

if (size(T) == [4,4] & size(s) == [6,1])
    TMP = [     T(1,1) * s(1) + T(1,2) * s(2) + T(1,3) * s(3); 
				T(2,1) * s(1) + T(2,2) * s(2) + T(2,3) * s(3); 
				T(3,1) * s(1) + T(3,2) * s(2) + T(3,3) * s(3) ];
    R = [   TMP(1); TMP(2); TMP(3);
			T(2,4) * TMP(3) - T(3,4) * TMP(2) + T(1,1) * s(4) + T(1,2) * s(5) + T(1,3) * s(6);
			T(3,4) * TMP(1) - T(1,4) * TMP(3) + T(2,1) * s(4) + T(2,2) * s(5) + T(2,3) * s(6);
			T(1,4) * TMP(2) - T(2,4) * TMP(1) + T(3,1) * s(4) + T(3,2) * s(5) + T(3,3) * s(6)];
    return;
elseif (size(T) == [3,1] & size(s) == [6,1])
    R = [   s(1); 
		    s(2); 
			s(3); 
			T(2) * s(3) - T(3) * s(2) + s(4); 
			T(3) * s(1) - T(1) * s(3) + s(5); 
			T(1) * s(2) - T(2) * s(1) + s(6) ];
    return;
else error('Adjoint Mapping needs first element for SE(3) or Vec(3) and second element for se(3)');
end;
            
