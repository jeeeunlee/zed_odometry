clc; clear; close all;

addpath('data');
addpath('../');
addpath('../matlab');
global repro_err lambda_error_max

% cell 형태로 데이터 받기 
data_l=[]; data_r=[];
for i=0:6
    fl = sprintf('feature2d_l_%d.txt',i);
    fr = sprintf('feature2d_r_%d.txt',i);
    data_l_i = load(fl);
    data_r_i = load(fr);
    data_l = [data_l;data_l_i];
    data_r = [data_r;data_r_i];
end

% ZED_HD
CamInfo = getCameraInfo('zed_HD');
ProjectionMat1 = CamInfo.K1 * CamInfo.T1; %3x4
ProjectionMat2 = CamInfo.K2 * CamInfo.T2; %3x4
repro_err=3; lambda_error_max=1.5;
repro_err=100; lambda_error_max=100;
[data_l_, data_r_, point3d_original] = triangulate(data_l, data_r, ProjectionMat1, ProjectionMat2);

repro_err=100; lambda_error_max=100;
Pl=ProjectionMat1; Pr=ProjectionMat2; niter=1;
for i=1:niter
    % eliminate outliers and get 3d points

    [data_l, data_r, point3d] = triangulate(data_l_, data_r_, Pl, Pr);
    % drawStereo(data_l, data_r, CamInfo);

    % optimization
%     alpha=1e-4;
%     alpha=5e-4;
    alpha=5e-8;
    % alpha=1e-8;
    Pl = EstimateProjectionMatrix(point3d, data_l, alpha);
    Pr = EstimateProjectionMatrix(point3d, data_r, alpha);
end
%% check reprojection
[data_l, data_r, point3d_] = triangulate(data_l, data_r, Pl, Pr);
x3b_ = [point3d_,ones(length(point3d_),1)];
x3b = [point3d_original,ones(length(point3d_original),1)];

xleft_projected = getReprojection(Pl,x3b_);
xleft_projected_original = getReprojection(ProjectionMat1,x3b);

xright_projected = getReprojection(Pr,x3b_);
xright_projected_original = getReprojection(ProjectionMat2,x3b);

figure()
subplot(4,1,1:2);
plot(data_l_(:,1),data_l_(:,2),'ko'); hold on;
plot(xleft_projected(:,1),xleft_projected(:,2),'rx');
plot(xleft_projected_original(:,1),xleft_projected_original(:,2),'bx');
xlim([0,1280]); ylim([0,720]); plot([0,1280],[CamInfo.cy,CamInfo.cy],'k-'); plot([CamInfo.cx,CamInfo.cx],[0,720],'k-');
error_left = xleft_projected-data_l;
error_left_original = xleft_projected_original-data_l_;
subplot(4,1,3);
plot(error_left(:,1),'r'); hold on;
plot(error_left_original(:,1),'b');
subplot(4,1,4);
plot(error_left(:,2),'r'); hold on;
plot(error_left_original(:,2),'b');


figure()
subplot(4,1,1:2);
plot(data_r_(:,1),data_r_(:,2),'ko'); hold on;
plot(xright_projected(:,1),xright_projected(:,2),'rx');
plot(xright_projected_original(:,1),xright_projected_original(:,2),'bx');
xlim([0,1280]); ylim([0,720]); plot([0,1280],[CamInfo.cy,CamInfo.cy],'k-'); plot([CamInfo.cx,CamInfo.cx],[0,720],'k-');
error_right = xright_projected-data_r;
error_right_original = xright_projected_original-data_r_;
subplot(4,1,3);
plot(error_right(:,1),'r'); hold on;
plot(error_right_original(:,1),'b'); legend('optimal','original')
subplot(4,1,4);
plot(error_right(:,2),'r'); hold on;
plot(error_right_original(:,2),'b'); legend('optimal','original')

Pl = Pl/Pl(3,3)
Pr = Pr/Pr(3,3)