function w = LogSO3(R)

SR_EPS = 1e-6;
SR_PI_SQRT2 = 2.22144146907918312351;

theta = 0.5* ( R(1,1) + R(2,2) + R(3,3) - 1 );

if theta < SR_EPS - 1.0
    w = zeros(3,1);
    if R(1,1) > 1.0 - SR_EPS
       w(1) = pi;
    elseif R(2,2) > 1.0 - SR_EPS
        w(2) = pi;
    elseif R(3,3) > 1.0 - SR_EPS
        w(3) = pi;
    else
        w(1) = SR_PI_SQRT2 * sqrt(  (R(2,1)^2 + R(3,1)^2) / (1 - R(1,1)));
        w(2) = SR_PI_SQRT2 * sqrt(  (R(1,2)^2 + R(3,2)^2) / (1 - R(2,2)));
        w(3) = SR_PI_SQRT2 * sqrt(  (R(1,3)^2 + R(2,3)^2) / (1 - R(3,3)));
    end    
else
    if theta > 1.0
        theta = 1.0;
    elseif theta < -1.0
        theta = -1.0;
    end
    theta = acos(theta);
    
    if theta < SR_EPS
        t_st = 3.0 / (6.0 - theta * theta);
    else
        t_st = theta/ (2.0 * sin(theta) );
    end
    w = t_st * [ R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2) ];
end
		
	