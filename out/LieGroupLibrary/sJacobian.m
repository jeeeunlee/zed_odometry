function J = SJacobian(theta)
% Space Jacobian for 5, 6, 7, d.o.f 

global A1 A2 A3 A4 A5 A6 A7 M

if (size(theta) == [7, 1])
    J(:, 1) = A1;
    J(:, 2) = Adj(expm(se3(A1)*theta(1)), A2);
    J(:, 3) = Adj(expm(se3(A1)*theta(1))*expm(se3(A2)*theta(2)), A3);
    J(:, 4) = Adj(expm(se3(A1)*theta(1))*expm(se3(A2)*theta(2))*expm(se3(A3)*theta(3)), A4);
    J(:, 5) = Adj(expm(se3(A1)*theta(1))*expm(se3(A2)*theta(2))*expm(se3(A3)*theta(3))*expm(se3(A4)*theta(4)), A5);
    J(:, 6) = Adj(expm(se3(A1)*theta(1))*expm(se3(A2)*theta(2))*expm(se3(A3)*theta(3))*expm(se3(A4)*theta(4))*expm(se3(A5)*theta(5)), A6);
    J(:, 7) = Adj(expm(se3(A1)*theta(1))*expm(se3(A2)*theta(2))*expm(se3(A3)*theta(3))*expm(se3(A4)*theta(4))*expm(se3(A5)*theta(5))*expm(se3(A6)*theta(6)), A7);
    return;
elseif (size(theta) == [6, 1])
    J(:, 1) = A1;
    J(:, 2) = Adj(expm(se3(A1)*theta(1)), A2);
    J(:, 3) = Adj(expm(se3(A1)*theta(1))*expm(se3(A2)*theta(2)), A3);
    J(:, 4) = Adj(expm(se3(A1)*theta(1))*expm(se3(A2)*theta(2))*expm(se3(A3)*theta(3)), A4);
    J(:, 5) = Adj(expm(se3(A1)*theta(1))*expm(se3(A2)*theta(2))*expm(se3(A3)*theta(3))*expm(se3(A4)*theta(4)), A5);
    J(:, 6) = Adj(expm(se3(A1)*theta(1))*expm(se3(A2)*theta(2))*expm(se3(A3)*theta(3))*expm(se3(A4)*theta(4))*expm(se3(A5)*theta(5)), A6);
    return;
elseif (size(theta) == [5, 1])
    J(:, 1) = A1;
    J(:, 2) = Adj(expm(se3(A1)*theta(1)), A2);
    J(:, 3) = Adj(expm(se3(A1)*theta(1))*expm(se3(A2)*theta(2)), A3);
    J(:, 4) = Adj(expm(se3(A1)*theta(1))*expm(se3(A2)*theta(2))*expm(se3(A3)*theta(3)), A4);
    J(:, 5) = Adj(expm(se3(A1)*theta(1))*expm(se3(A2)*theta(2))*expm(se3(A3)*theta(3))*expm(se3(A4)*theta(4)), A5);
    J(:, 6) = Adj(expm(se3(A1)*theta(1))*expm(se3(A2)*theta(2))*expm(se3(A3)*theta(3))*expm(se3(A4)*theta(4))*expm(se3(A5)*theta(5)), A6);
    return;
else
    error('theta = 7 or 6 or 5');
end
