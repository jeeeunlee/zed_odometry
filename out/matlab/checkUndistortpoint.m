clc; clear all; close all;
addpath('../');

pnts2D = load('mvKeys.txt');
undistortpnts2D = load('mvKeysUn.txt');

% ZED_HD
CamInfo = getCameraInfo('zed_HD');

undistortpnts2D_ = [undistortpnts2D, ones(length(undistortpnts2D),1)] * CamInfo.K1';

figure()
plot(pnts2D(:,1),pnts2D(:,2),'.'); hold on; xlim([0,1280]); ylim([0,720])
plot(undistortpnts2D_(:,1),undistortpnts2D_(:,2),'x'); plot([0,1280],[CamInfo.cy,CamInfo.cy],'k-'); plot([CamInfo.cx,CamInfo.cx],[0,720],'k-');
figure()
plot(undistortpnts2D(:,1),undistortpnts2D(:,2),'x'); xlim([-1,1]); ylim([-0.5,0.5])


figure()
plot(pnts2D(:,1),pnts2D(:,2),'.'); hold on; xlim([0,1280]); ylim([0,720]); plot([0,1280],[CamInfo.cy,CamInfo.cy],'k-'); plot([CamInfo.cx,CamInfo.cx],[0,720],'k-');
plot(undistortpnts2D(:,1),undistortpnts2D(:,2),'x'); 