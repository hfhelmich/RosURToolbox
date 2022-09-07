%% SCRIPT_DH_SphericalPrint
% This script provides a generic template for the DH parameters required
% for a spherical print.
%
%   M. Kutzer, 12Jul2021, USNA

clear all
close all
clc

%% Define parameters
%thetas = deg2rad( [45; -60; 30+90; 0] );
thetas = deg2rad( [30; -45; 90+100; 0] );
ds = [100; 80; 60; 40];
as = [100; 80; 60; 0];
alphas = [0; 0; pi/2; 0];

DHtable = [thetas, ds, as, alphas];

fig = figure;
axs = axes('Parent',fig);
hold(axs,'on');
daspect(axs,[1 1 1]);

h = plotDHtable(axs,DHtable);

%% Adjust view
view(axs,2);
setTriad(h,'Scale',25);

%% Define sphere
%r = 20;      % Sphere radius (mm)
%ds = 60;   % Stem length (mm)
r = 10;
ds = 20;
sfit.Center = zeros(3,1);
sfit.Radius = r;
ptch = patch( patchSphere(sfit,1000),'EdgeColor','None','FaceColor','b',...
    'FaceAlpha',0.5 );
h_Sto4 = triad('Parent',h(end),'Matrix',Tz(ds),'Scale',18,'LineWidth',1.5);
set(ptch,'Parent',h_Sto4);

%% Save figure
set(fig,'Units','Inches','Position',[1,1,12.67,5.4],'Color',[1,1,1]);
set(axs,'Visible','off');

%saveas(fig,'SpherePrintDH.png','png');

%% Test spherical print inverse kinematics
x_Nto0 = DHtableToFkin(DHtable)*Tz(ds)*[0;0;0;1] + [0;r;0;0];
phi = sum( DHtable(1:3,1) ) - pi/2;
plt = plot3(axs,x_Nto0(1),x_Nto0(2),x_Nto0(3),'*m');

%% Animate
sol = 'elbow-up, wrist-up';
%sol = 'elbow-down, wrist-up';
az = 0;
for el = linspace(0,pi,1000)
    q = SpherePrintDH_ikin([az,el],ds,r,x_Nto0,DHtable,sol);
    
    if ~isempty(q)
        DHtable(:,1) = q;
        
        for i = 1:4
            H_i2j = DH(DHtable(i,1),DHtable(i,2),DHtable(i,3),DHtable(i,4));
            set(h(i+1),'Matrix',H_i2j);
        end
        drawnow;
    end
end


