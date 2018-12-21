function output = so3(input)
% so(3) matrix form converting from 3*1 to 3*3
% R = so3(v)
% v : 3*1, R : 3*3
% LieGroup Matlab Library [Syungkwon Ra, dearLenin@gmail.com]

if (size(input) == [3,1])
    output = [    0    -input(3)  input(2);
             input(3)  0    -input(1);
            -input(2)  input(1)  0   ];
    return;
elseif (size(input) == [3,3])
    theta = acos((trace(input)-1)/2);
    if abs(theta) < 10^-10
        output = zeros(3,1);
    else
        w_hat = (input-input')/(2*sin(theta))*theta;
        output(1,1) = w_hat(3,2);
        output(2,1) = w_hat(1,3);
        output(3,1) = w_hat(2,1);
    end    
    return;
else error('check the size of matrix');
end;