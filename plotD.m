function plotD(xmin,xmax,res)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file                Author: Ricardo Sanfelice
%
% Project: Simulation of a hybrid system
%
%
% Name: plotD.m
%
% Description: plots the jump set
%
% Version: 0.6
% Required files: -
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% get dimensions
d = length(xmin);
%figure(3)
%clf

if (d==1)
   xin = linspace(xmin,xmax,res);
   N = length(xin);
   for i=1:N,   
%     Cout(i) = C(xin(i));
     if (D(xin(i)) == 1)
          plot(xin(i),'r*')
          hold on
     end
   end
%elseif (d==2)
end