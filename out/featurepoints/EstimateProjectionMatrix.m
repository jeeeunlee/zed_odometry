function P = EstimateProjectionMatrix(point3d, point2d, alpha)
n=length(point3d); 
x2c = reshape(point2d', 2*n, 1);
x3b = [point3d,ones(length(point3d),1)];
A = [repmat(-eye(2), n, 1), x2c];
B = reshape(repmat(x3b,1,2)',4,2*n);

cvx_begin sdp
    variable r(1)
    variable Q3(4,4) semidefinite
    variable P(3,4)
    minimize (r + alpha*trace(Q3))
    subject to
    P(1,1) == 1e-5;
    P(1,2) <= 7e-5;
    P(2,1) <= 7e-5;
    P(1,2) >= -7e-5;
    P(2,1) >= -7e-5;
    [1, P(3,:); P(3,:)', Q3] >= 0 ;%== semidefinite(5);
    [diag(diag(B'*Q3*B)), diag(A*P*B)  ;zeros(1,2*n), r] >= 0; %== semidefinite(2*n+1);
cvx_end