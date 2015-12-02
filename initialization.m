% initialization for UAV Loitering project

clear all; close all; clc;
PrintPics = true;
Scenario = 'Inside';                    

% initial conditions
switch Scenario
    case 'Outside'
        X0 = 3000;%m
        Y0 = 3000; %m
        rc = 350; %m
        q0 = 2;
        T = 80;
    case 'Inside'
        X0 = 0;%m
        Y0 = 0; %m
        rc = 2500; %m
        q0 = 1;
        T = 80;
    case 'Donut'
        X0 = 0;%m
        Y0 = 0; %m
        rc = 5000; %m
        q0 = 2;
        T = 120;
end

V0 = 175;%m/sec
Psi0 =0* 45*pi/180; %rad
x0 = [X0; Y0; V0; Psi0; q0];                                                             
u0 = 1;
P = 1; %CW rotation direction
Mu = 0.1; %hysteresis for changing rotation direction

% combine initial conditions                                            
z0 = [x0; u0];                                                          

%UAV parameters (Reaper UAV)
Mass = 2500; %kg (TOGW = 4760 kg) 
cLmax = 0.85; %guess
cD0 = 0.002; %guess (calculated based on cruise speed)
e = 0.99; %guess
AR = 19; %rough calculation
k = 1/(pi*e*AR);
sref = 35.15; %m^2 (rough calculation)
maxThrust = 9000; %N (based on 671 kW at 300 km/h
minThrust = 50; %N
%maxThrust = 2000; %N

% Environment variables
gravity = 9.81;  %m/sec^2 gravity constant
rho = 1.112;  % kg/m^3 air density (1000 m  altitude)

%commanded radius and acceleration
vc1 = 160; %m/s
vc2 = 200; %m/s


%intermediate Calculation
Wt = Mass*gravity; 
phic = atan2(vc1^2,gravity*rc);
qc = 0.5*rho*vc1^2;

[Drag_c Phi_maxT] = CalculateDrag(phic,qc,cD0,sref,Wt,k,maxThrust);
Vmin = sqrt(2*Wt/(sref*rho*cLmax));
Vmax = sqrt( (maxThrust + sqrt(maxThrust^2-(16*cD0*Wt^2)/(pi*e*AR)))/(rho*cD0) );
Phimax = acos(2*Wt/(sref*rho*cLmax*Vmax^2));
Phimax =min(Phi_maxT,Phimax);

%hybrid system switch parameters
C = 0.5*(pi^2+(Vmax-Vmin)^2)+1;
D = sqrt(2*C)+200;

%error check
VmaxLD = sqrt(2*Wt*sqrt(k/cD0)/(rho*sref));
Rmin = VmaxLD^2/( gravity*sqrt((cLmax*sref*rho*VmaxLD^2/(2*Wt))^2-1) );

if V0 <= Vmin
    error(['Initial velocity of ',num2str(V0),' m/s is less than Vmin of ',num2str(Vmin),' m/s'])
end
if min(vc1,vc2) <= Vmin
    error(['Commanded velocity of ',num2str(min(vc1,vc2)),' m/s is less than Vmin of ',num2str(Vmin),' m/s'])
end
if rc < Rmin
    error(['Commanded radius of ',num2str(rc),' m is less than Rmin of ',num2str(Rmin),' m'])
end
if Drag_c > maxThrust
    error(['Required Thrust of ',num2str(Drag_c),' N is greater than Tmax of ',num2str(maxThrust),' N'])
end

% simulation horizon                                                    
                                                                 
J = 5;                                                                 
                                                                        
% rule for jumps                                                        
% rule = 1 -> priority for jumps                                        
% rule = 2 -> priority for flows                                        
% rule = 3 -> no priority, random selection when simultaneous conditions
rule = 1;                                                               
                                                                        
% constants                                                                                                                        
n = 5; %# of state components                                       
m = 1; %# of input components  

%solver tolerances
RelTol = 1e-8;
MaxStep = .005;

% simulate
sim('HybridSimulator')