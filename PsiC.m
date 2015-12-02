function [Psi_c,p] = PsiC(X,Y,V,Psi,q,p,Rc,mu)
R = sqrt(X^2 + Y^2);
Pos = [X Y];
PcrossV = det([Pos' [V*cos(Psi) V*sin(Psi)]']); %cross product for 2x2
tilde = sign(Rc-R);

%Initialize to current heading
Psi_c = Psi;
%Determine Desired Heading
if R ~= 0
    switch q
        case 1 %local
            %switch rotation direction if necessary
            if( (PcrossV < -mu && p == 1) || (PcrossV > mu && p == -1) );
                p = -p;
            end
            %Heading approaching loiter circle at tangent
            top = Y*(Rc^2 - R^2) + p*2*X*R*Rc;
            bottom = X*(Rc^2 - R^2) - p*2*Y*R*Rc;
            Psi_c = atan2(top,bottom);

        case 2 %global
            %Heading for center of loiter circle
            Psi_c = atan2(tilde*Y,tilde*X);

    end
end
%limit Psi_c to +/-180
while Psi_c < -pi
    Psi_c = Psi_c + 2*pi;
end
while Psi_c > pi
    Psi_c = Psi_c - 2*pi;
end
Psi_c = rem(Psi_c,2*pi);