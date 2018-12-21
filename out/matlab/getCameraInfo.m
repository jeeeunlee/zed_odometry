function CamInfo = getCameraInfo(model)

switch(model)
    case 'zed_HD'
        CamInfo.fx= 699.739; CamInfo.fy= 699.739; CamInfo.cx= 634.368; CamInfo.cy= 339.231;
        CamInfo.rfx= 700.365; CamInfo.rfy= 700.365; CamInfo.rcx= 628.349; CamInfo.rcy= 334.552;
        CamInfo.CV= 0.00918222 ; CamInfo.RX= -0.00153492; CamInfo.RZ= -0.000304831;
        CamInfo.width = 1280;
        CamInfo.height = 720;
        
    case 'zed_2K'
    case 'zed_FHD'
    case 'zed_VGA'
        
end

w = [CamInfo.RX, CamInfo.CV, CamInfo.RZ]; w_norm = norm(w); w_axis = w/w_norm;

CamInfo.R = axang2rotm([w_axis, w_norm]);
% CamInfo.R = eye(3);
CamInfo.R = CamInfo.R';
CamInfo.t = [0.12;0.0;0.0];
CamInfo.T1 = [eye(3),zeros(3,1)];
CamInfo.T2 = [CamInfo.R', -CamInfo.R'*CamInfo.t];
CamInfo.K1 = [CamInfo.fx,0,CamInfo.cx; 0,CamInfo.fy,CamInfo.cy;0,0,1];
CamInfo.K2 = [CamInfo.rfx,0,CamInfo.rcx; 0,CamInfo.rfy,CamInfo.rcy;0,0,1];

end