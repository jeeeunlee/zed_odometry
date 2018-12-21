clc; clear all; close all;
addpath('../');
addpath('../LieGroupLibrary/');

tvec = load('tvec.txt');
rvec = load('rvec.txt');

t_accum = [0;0;0];
t_accum2 = [0;0;0];
for i=1:length(tvec)-48
    R = LargeSO3(rvec(i,:));
    t = -R'*tvec(i,:)';
    t_accum = [t_accum, t_accum(:,end)+t];
    t_accum2 = [t_accum2, t_accum2(:,end)+tvec(i,:)'];
end

t_accum(:,end-10:end)
plot3(t_accum(1,:), t_accum(2,:), t_accum(3,:));grid on; axis equal;

figure();
plot3(t_accum2(1,:), t_accum2(2,:), t_accum2(3,:));grid on;axis equal;