function y = metric(T, T_0)

if ((size(T) == [4,4]) & size(T_0) == [4,4])
    y = zeros(2,1);
    y(1) = norm(logm(T(1:3, 1:3)'*T_0(1:3, 1:3)), 1);
    y(2) = norm( T(1:3,4) - T_0(1:3, 4), 1);
else
    error('T, T_0 : se3()')
end
