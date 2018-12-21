function w = Large_SO3(R)
% SO(3) matrix form converting from 3*3 to 3*1
% R = Large_SO3(v)
% w : 3*1, R : 3*3

w = zeros(3,1);

if (size(R) == [3,3])
    
    theta = acos((trace(R)-1)/2);
    if abs(theta) < 10^-10
        w = zeros(3,1);
    else
        w_hat = (R-R')/(2*sin(theta))*theta;
        w(1) = w_hat(3,2);
        w(2) = w_hat(1,3);
        w(3) = w_hat(2,1);
    end
  
    return;
else error('Elements must have size of 3 row and 1 column');
end;