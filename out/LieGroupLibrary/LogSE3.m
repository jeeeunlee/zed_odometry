function v = LogSE3(T)

SR_EPS = 1e-6;
SR_PI_SQRT2 = 2.22144146907918312351;
SR_PI_SQR  = 9.86960440108935861883;
theta = 0.5* ( T(1,1) + T(2,2) + T(3,3) - 1 );

if theta < SR_EPS - 1.0
    w = zeros(3,1);
    if T(1,1) > 1.0 - SR_EPS
       w(1) = pi;
    elseif T(2,2) > 1.0 - SR_EPS
        w(2) = pi;
    elseif T(3,3) > 1.0 - SR_EPS
        w(3) = pi;
    else
        w(1) = SR_PI_SQRT2 * sqrt(  (T(2,1)^2 + T(3,1)^2) / (1 - T(1,1)));
        w(2) = SR_PI_SQRT2 * sqrt(  (T(1,2)^2 + T(3,2)^2) / (1 - T(2,2)));
        w(3) = SR_PI_SQRT2 * sqrt(  (T(1,3)^2 + T(2,3)^2) / (1 - T(3,3)));
    end
    w2 = w.*w;
    w3 = w2.*w;
    v = w([2,3,1]).*w([3,1,2]);
    id = 0.25 * SR_PI_SQR / (w2'*w2 + 2 * w2'* w2([2,3,1]));
    p = T(1:3,4) * id;
    
    v = [ w ;  
        2.0 * ( 2.0 * w2(1) * p(1) + ( w3(3) + v(2) * w(1) + v(1) * w(2) + 2.0 * v(3) ) * p(2) + (2.0 * v(2) - w3(2) - w(1) * v(3) - w(3) * v(1) ) * p(3) );
        2.0 * ((2.0 * v(3) - w3(3) - v(1) * w(2) - v(2) * w(1) ) * p(1) + 2.0 * w2(2) * p(2) + (w3(1) + w(3) * v(2) + v(3) * w(2) + 2.0 * v(1) ) * p(3) );
        2.0 * ((w(1) * v(3) + w(3) * v(1) + 2.0 * v(2) + w3(2) ) * p(1) + (2.0 * v(1) - w3(1) - v(3) * w(2) - w(3) * v(2) ) * p(2) + 2.0 * w2(3) * p(3) )];
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
    st = sin(theta);
    w = [ T(3,2) - T(2,3); T(1,3) - T(3,1); T(2,1) - T(1,2) ];
    w2 = w.*w;
    w3 = w([2,3,1]) .* w([3,1,2]);
    
    w = t_st * w;
    
    if theta < SR_EPS
        stct = theta/48.0;
    else
        stct = (2.0*st - theta * (1.0 + cos(theta)))/ (8.0 * st * st * st);
    end
    
    v = [ w;
        (1.0 - stct * (w2(2) + w2(3))) * T(1,4) + ( 0.5 * w(3) + stct * w3(3)) * T(2,4) + (stct * w3(2) - 0.5 * w(2)) * T(3,4) ;
        (stct * w3(3) - 0.5 * w(3) ) * T(1,4) + (1.0 - stct * (w2(1) + w2(3))) * T(2,4) + (0.5 * w(1) + stct * w3(1)) * T(3,4) ;
        (0.5 * w(2) + stct * w3(2)) * T(1,4) + (stct * w3(1) - 0.5 * w(1)) * T(2,4) + (1.0 - stct * (w2(2) + w2(1))) * T(3,4)];
    
    
end
		
	