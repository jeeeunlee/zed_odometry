addpath('../');

xleft = load('left.txt');
xright = load('right.txt');
X = load('p3.txt');
output = load('output.txt');

n = min([length(xleft), length(xright), length(X)]);

% ZED_HD
CamInfo = getCameraInfo('zed_HD');
% CamInfo.T2 = [eye(3),[-0.12;0;0]];

ProjectionMat1 = CamInfo.K1 * CamInfo.T1;
ProjectionMat2 = CamInfo.K2 * CamInfo.T2;


Xbar = [X'; ones(1,n)];
xleft_projected = (ProjectionMat1*Xbar)';
xleft_projected = xleft_projected(:,1:2)./xleft_projected(:,3);
xright_projected = (ProjectionMat2*Xbar)';
xright_projected = xright_projected(:,1:2)./xright_projected(:,3);

outputL_projected = (ProjectionMat1*output')';
outputR_projected = (ProjectionMat2*output')';


% [xleft, xleft_projected, outputL_projected]
% [xright, xright_projected, outputR_projected]

close all;
figure()
subplot(3,1,1:2);
plot(xleft(:,1),xleft(:,2),'bo'); hold on;
plot(xleft_projected(:,1),xleft_projected(:,2),'rx');
xlim([0,1280]); ylim([0,720]); plot([0,1280],[CamInfo.cy,CamInfo.cy],'k-'); plot([CamInfo.cx,CamInfo.cx],[0,720],'k-');
error_left = xleft_projected-xleft;
subplot(3,1,3);
plot(error_left);

% plot(outputL_projected(:,1),outputR_projected(:,2),'rx')

figure()
subplot(3,1,1:2);
plot(xright(:,1),xright(:,2),'bo'); hold on; 
plot(xright_projected(:,1),xright_projected(:,2),'rx')
xlim([0,1280]); ylim([0,720]); plot([0,1280],[CamInfo.cy,CamInfo.cy],'k-'); plot([CamInfo.cx,CamInfo.cx],[0,720],'k-');
error_right = xright_projected-xright;
subplot(3,1,3);
plot(error_right);
