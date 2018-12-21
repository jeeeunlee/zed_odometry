function M_ = functionM(gamma, Q3, P, point2d, point3d)
nn = length(point2d);
M = zeros(2*nn+1);
for j=1:nn
    M(2*j-1:2*j,2*j-1:2*j) = [point3d(j,:),1] * Q3 * [point3d(j,:),1]' * eye(2);
    M(2*j-1:2*j,2*nn+1) = (point3d(j,:)' * P(3,:) - P(1:2,:))*[point3d(j,:),1]';
end
M(2*nn+1,2*nn+1) = gamma;

M_ = M;
end