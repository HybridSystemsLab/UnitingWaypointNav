%Local Controller check
clear all;close all;clc;

theta = -pi:0.1:pi;
alpha = 0.13;
% Cps = cos(theta) + sin(theta);
% Cms = cos(theta) - sin(theta);
% ts = theta.^2;

test = cos(theta) + alpha*theta.^2;

plot(theta,test,'b')
xlabel('\theta (rad)')
grid on