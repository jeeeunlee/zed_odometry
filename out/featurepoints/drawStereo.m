function drawStereo(data_l, data_r, CamInfo)

figure();
% for i=1:length(data_l)
%     plot([data_l(i,1), data_r(i,1)+CamInfo.width] , [data_l(i,2), data_r(i,2)], 'k-'); hold on;
%     plot(data_l(i,1),data_l(i,2) , 'ro'); hold on;
%     plot(data_r(i,1)+CamInfo.width,data_r(i,2) , 'bo'); hold on;
% end
% axis equal; xlim([0,2*CamInfo.width]); ylim([0,CamInfo.height]);grid on; 

for i=1:length(data_l)
    plot([data_l(i,1), data_r(i,1)] , [data_l(i,2), data_r(i,2)], 'k-'); hold on;
    plot(data_l(i,1),data_l(i,2) , 'ro'); hold on;
    plot(data_r(i,1),data_r(i,2) , 'bo'); hold on;
end
axis equal; xlim([0,CamInfo.width]); ylim([0,CamInfo.height]);grid on; 
