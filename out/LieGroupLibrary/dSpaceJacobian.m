% build time derivative of space jacobian : dJ_s
% dJ_s = dSpaceJacobian(J_s, dq, n)
% input variables
% J_s   : space jacobian
% dq    : joint angular velocity
% n     : number of joint

function dJ_s = dSpaceJacobian(J_s, dq, n)

% dJ_s = zeros(6,n);
% for i = 2:1:n
%     temp = zeros(6,1);
%     for j =1:1:i-1
%         temp = temp + ad(J_s(:,i), J_s(:,j))*dq(j);
%     end
%     dJ_s(:,i) = temp;
% end

for i=1:1:n
    temp = zeros(6,1);
    for j=1:1:n
        if i > j, temp = temp + ad(J_s(:,i), J_s(:,j))*dq(j);, end
    end
    dJ_s(:,i) = temp;
end