function R = Adjm(T)
% Adjoint mapping of SE(3) T and se(3) s
% Adjoint mapping of vec3 T and se(3) s (T's orientation is I)
% R = Adj(T, s)
% T : 4*4 or 3*1, s : 6*1, R : 6*1
% LieGroup Matlab Library [Syungkwon Ra, dearLenin@gmail.com]

if (size(T) == [4,4])
    tmp = [ T(2,4)*T(3,1) - T(3,4)*T(2,1)   T(2,4)*T(3,2) - T(3,4)*T(2,2)   T(2,4)*T(3,3) - T(3,4)*T(2,3);
            T(3,4)*T(1,1) - T(1,4)*T(3,1)   T(3,4)*T(1,2) - T(1,4)*T(3,2)   T(3,4)*T(1,3) - T(1,4)*T(3,3);
            T(1,4)*T(2,1) - T(2,4)*T(1,1)   T(1,4)*T(2,2) - T(2,4)*T(1,2)   T(1,4)*T(2,3) - T(2,4)*T(1,3)];
    R = [ T(1:3,1:3)    zeros(3);
          tmp           T(1:3,1:3)];
    return;
elseif (size(T) == [3,1])
%     tmp = expm(so3(T));
    tmp = LargeSO3(T);
    R = [ tmp       zeros(3);
          zeros(3)  tmp];
    return;
else error('Adjoint Mapping needs first element for SE(3) or Vec(3) and second element for se(3)');
end;
            
