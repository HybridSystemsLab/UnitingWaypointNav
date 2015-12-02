%postprocessing for the Loitering UAV
close all
%x0 = [X0; Y0; V0; Psi0; q0; T0; Phi0;];
%U = [Phi; T; Psi_c; Drag];
figtype = {'png'};                  
Msize = 10;

Q = x(:,5);
Vmag = x(:,3);
PhiMax = acos(2*Wt./(sref*rho*cLmax.*Vmag.^2));
Vmin = sqrt(2*Wt./(sref.*rho.*cLmax.*cos(U(:,1))));
r2d = 180/pi;
q = 0.5*rho*Vmag.^2;

CL = Wt./(sref.*q.*cos(U(:,1)));
CD  = cD0 + k.*CL.^2;
%Drag = q.*sref.*CD;
Drag = U(:,4);
Lift = q.*sref.*CL;

%limit Psi_c to +/-180
Psi_c = U(:,3);
for i=1:length(Psi_c)
    while Psi_c(i) < -pi
        Psi_c(i) = Psi_c(i) + 2*pi;
    end
    while Psi_c(i) > pi
        Psi_c(i) = Psi_c(i) - 2*pi;
    end
    Psi_c(i) = rem(Psi_c(i),2*pi);
end

%limit Psi to +/-180
Psi =x(:,4);
for i=1:length(Psi)
    while Psi(i) < -pi
        Psi(i) = Psi(i) + 2*pi;
    end
    while Psi(i) > pi
        Psi(i) = Psi(i) - 2*pi;
    end
    Psi(i) = rem(Psi(i),2*pi);
end

r= sqrt(x(:,2).^2+x(:,1).^2);
zeta = 10*pi/180;
Error_mag = abs(r - rc) + abs(Vmag - vc1) + abs(Psi - Psi_c);
epsilon = Vmag.*abs(r - rc)./(rc^2+r.^2).*2.*r.*rc.*abs(sin(Psi - Psi_c)).*(1- sin(Psi - Psi_c)/zeta);
epsilon(Q == 2 | abs(Psi - Psi_c) >= zeta | sign(r-rc).*sign(sin(Psi - Psi_c)).*P > 0) = nan;


h=figure('Name','Simulation Results','NumberTitle','off');
% [left bottom width height]
set(h, 'Position', [75   450   1800   504]);
subplot(1,3,1)
ht = plot(x(:,2),x(:,1),'b'); %this will be overwritten and replotted

if ~strcmp(Scenario,'Outside')
    axis equal
end

%Local Control Region
xbnd = get(gca,'Xlim');
ybnd = get(gca,'Ylim');
if ~strcmp(Scenario,'Outside')
    if abs(xbnd(2)-rc) < D && abs(ybnd(2)-rc) < D
        xbnd(1) = -1.1*(D+rc);
        xbnd(2) = -xbnd(1);
        ybnd(1) = -1.1*(D+rc);
        ybnd(2) = -ybnd(1);
    end
end
xgrid = linspace(xbnd(1),xbnd(2),100);
ygrid = linspace(ybnd(1),ybnd(2),101);
for idx1=1:length(xgrid)
    for idx2=1:length(ygrid)
        R_minus_Rc(idx1,idx2) = sqrt(xgrid(idx1)^2 + ygrid(idx2)^2)-rc;
    end
end


[c,hc1] = contourf(xgrid,ygrid,-abs(R_minus_Rc'),-[D D],'LineStyle','None');
alphable1 = findobj(hc1, '-property', 'FaceAlpha','-and','-property', 'FaceColor','-and','-not','FaceColor',[1 1 1]);
background1 = findobj(hc1,'-property', 'FaceAlpha','-and','-property', 'FaceColor','-and','FaceColor',[1 1 1]);
set(alphable1, 'FaceAlpha', 0.3,'FaceColor',[0 0 1]);
%set(background1,'FaceAlpha',0.0,'FaceColor','w');
hold on; box on;
[~,hc2] = contourf(xgrid,ygrid,abs(R_minus_Rc'),[sqrt(2*C) sqrt(2*C)],'LineStyle','None');
alphable2 = findobj(hc2,'-property', 'FaceAlpha','-and','-property', 'FaceColor','-and','-not','FaceColor',[1 1 1]);
background2 = findobj(hc2,'-property', 'FaceAlpha','-and','-property', 'FaceColor','-and','FaceColor',[1 1 1]);
set(alphable2, 'FaceAlpha', 0.3,'FaceColor',[1 0 0]);
set(background2,'FaceAlpha',0.5,'FaceColor','w');

ht = plot(x(:,2),x(:,1),'b','LineWidth',2);
hrc = circle(0,0,rc);
set(hrc,'Color','r','LineWidth',2.5,'LineStyle','--')
xlabel('Y (m)')
ylabel('X (m)')
title('Position')
tempi = find(Q~=Q(1),1,'first');
if isempty(tempi)
    legend([ht hrc alphable1(1) findobj(alphable2,'FaceColor','r')],[{'Trajectory'},{'Loiter Radius'},{'C_1'},{'C_2'}],'Location','Best')
else
    hj = plot(x(tempi,2),x(tempi,1),'*b','MarkerSize',Msize);
    legend([ht hrc hj alphable1(1) alphable2(1)],[{'Trajectory'},{'Loiter Radius'},{'Controller Switch'},{'C_1'},{'C_2'}],'Location','Best')
end
grid on; box on;

subplot(1,3,2)
hold on; box on;
V_t = plot(t,Vmag,'LineWidth',2);
if isempty(tempi)
    if Q(1) == 2
        Vcom = [vc2 vc2];
        Vcomt = [t(1) t(end)];
    else
        Vcom = [vc1 vc1];
        Vcomt = [t(1) t(end)];
    end
    vcomh = plot(Vcomt, Vcom,'LineWidth',2,'Color','k');
    legend([V_t vcomh],[{'Velocity'},{'Commanded Velocity'}],'Location','Best')
else
    if Q(1) == 2
        Vcom = [vc2 vc2 nan vc1 vc1];
        Vcomt = [t(1) t(tempi) nan t(tempi) t(end)];
    else
        Vcom = [vc1 vc1];
        Vcomt = [t(1) t(end)];
    end
    swth = plot(t(tempi),Vmag(tempi),'*b','MarkerSize',Msize);
    vcomh = plot(Vcomt, Vcom,'LineWidth',2,'Color','k');
    legend([V_t swth vcomh],[{'Velocity'},{'Controller Switch'},{'Commanded Velocity'}],'Location','Best')
end
set(gca,'Ylim',[ min(vc1,vc2)-10 max(vc1,vc2)+10]);    
xlabel('time (s)')
ylabel('Velocity (m/s)')
grid on

subplot(1,3,3)
hold on; box on;
%put gap at controller switch
if isempty(tempi)
    time = t;
    thrust = U(:,2)*1e-3;
    bankAngle = U(:,1)*r2d;
else
    time = [t(1:tempi-1); t(tempi-1:end)];
    thrust = [U(1:tempi-1,2); nan; U(tempi:end,2)]*1e-3;
    bankAngle = [U(1:tempi-1,1); nan; U(tempi:end,1)]*r2d;
end
[ax PhiC_t Thrst_t] = plotyy(time,bankAngle,time,thrust);
hold on;
set([PhiC_t Thrst_t],'LineWidth',2);
if ~isempty(tempi)
    swth = plot(ax(1),[time(tempi-1) time(tempi-1)],[-90 90],'LineStyle','--','LineWidth',2,'Color','k');
    legend(swth,'Controller Switch')
end
xlabel('time (s)')
set(ax(1),'Ylim',[-90 90])
set(ax(1),'Ytick',-90:30:90)
if strcmp(Scenario,'Outside')
    set(ax(2),'Ytick',[0 2 5 8 10])
end
ylabel(ax(1),'\Phi (deg)')
ylabel(ax(2),'Thrust (kN)')
title('Control Inputs')
grid on


if 0 && PrintPics
    FileName = ['Sim_Results_' Scenario];
    for i=1:length(figtype)
        saveas(h,FileName,figtype{i});
    end
end

% h=figure('Name','Psi','NumberTitle','off');
% hold on; box on;
% plot(t,Psi*180/pi,'k');
% plot(t,Psi_c*180/pi,'b');
% legend('Psi','Psi Commanded','Location','Best')
% xlabel('time (s)')
% ylabel('Psi (degrees)')
% grid on
% if PrintPics
%     if sqrt(x(1,2)^2+x(1,1)^2)<=rc
%         FileName = 'Sim_Heading_In';
%     else
%         FileName = 'Sim_Heading';
%     end
%     for i=1:length(figtype)
%     print(h,figtype{i},FileName);
%     end
% end

% h=figure('Name','Error','NumberTitle','off');
% hold on; box on;
% plot(t,Error_mag,'LineWidth',2,'Color','k');
% plot(t,epsilon,'LineWidth',2,'LineStyle','--','Color','k');
% legend('|e_1|','\epsilon_{max}','Location','Best')
% xlabel('time (s)')
% ylabel('Error')
% grid on
% if PrintPics
%     if sqrt(x(1,2)^2+x(1,1)^2)<=rc
%         FileName = 'Sim_Error_In';
%     else
%         FileName = 'Sim_Error';
%     end
%     for i=1:length(figtype)
%     print(h,figtype{i},FileName);
%     end
% end