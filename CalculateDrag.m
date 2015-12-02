function [D Phi_maxT] = CalculateDrag(Phi,q,Cd0,S,W,K,Tmax)

D = Cd0*S*q + K*W^2/(q*S*cos(Phi)^2);

Phi_maxT = acos(sqrt(K*W^2 / (q*S*(Tmax-Cd0*q*S)) ));