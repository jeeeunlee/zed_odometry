function [point2d_l_a, point2d_r_a, point3d_a] = triangulate(point2d_l, point2d_r, ProjectionMat1, ProjectionMat2)
global repro_err lambda_error_max
n = min([length(point2d_l), length(point2d_r)]);
point3d = zeros(n,3); lambda_min = zeros(n,1);

point3d_a = []; point2d_l_a=[]; point2d_r_a=[];

% ProjectionMat1 = CamInfo.K1 * CamInfo.T1; %3x4
% ProjectionMat2 = CamInfo.K2 * CamInfo.T2; %3x4

% find x* satisfies Ax=0 by minimizing norm(Ax)

for i =1:n
    a1 = point2d_l(i,1) * ProjectionMat1(3,:) - ProjectionMat1(1,:);
    a2 = point2d_l(i,2) * ProjectionMat1(3,:) - ProjectionMat1(2,:);
    a3 = point2d_r(i,1) * ProjectionMat2(3,:) - ProjectionMat2(1,:);
    a4 = point2d_r(i,2) * ProjectionMat2(3,:) - ProjectionMat2(2,:);
    A = [a1;a2;a3;a4];
    [u,s,v] = svd(A);    
    lambda_min(i) = s(4,4); 
    
    x = v(:,4);
    point3d(i,:) = x(1:3)'/x(4);
end

lambda_error = max(mean(lambda_min)*1.2, lambda_error_max);
x3b = [point3d,ones(length(point3d),1)];
projected_l = getReprojection(ProjectionMat1,x3b);
projected_r = getReprojection(ProjectionMat2,x3b);
err_l = projected_l-point2d_l;
err_r = projected_r-point2d_r;

for i =1:n
     if(lambda_min(i) < lambda_error && norm([err_l(i,:),err_r(i,:)]) < repro_err )
        point3d_a = [point3d_a; point3d(i,:)];
        point2d_l_a = [point2d_l_a; point2d_l(i,:)];
        point2d_r_a = [point2d_r_a; point2d_r(i,:)];
    end
end

end