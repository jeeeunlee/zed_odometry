clc; clear all; close all;
addpath('../');

xleft = load('left.txt');
xright = load('right.txt');
X = load('p3.txt');
X_opencv = load('p3_opencv.txt');

plot(X); hold on; plot(X_opencv,'--')