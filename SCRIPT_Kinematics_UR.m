%% SCRIPT_Kinematics_UR
% Explore the URDF kinematics prescribed by the "sia20.urdf"
%
% M. Kutzer, 21Jun2021, USNA
clear all
close all
clc

%% Import URDF model
% Establish robot rigid body tree from ROS urdf and mesh stl's
% "Official Motoman Release"
robot = loadrobot("universalUR3"); % <--- Not added until 2021a
%robot = loadrobot("abbIrb120");
%robot = loadrobot("willowgaragePR2");

% Create config structure with home joint angle and angle names
config = homeConfiguration(robot);

% Show the robot in the home joint configuration
axs = show(robot,config,'Visuals','on','PreservePlot',false);

%% Show robot details
showdetails(robot)

%% Recover forward kinematics
[strs,funcs,links,jlims,jhome] = rigidBodyTree2fkin(robot);

%% Recover actual forward kinematics
fkin_str = strs{2};
fkin_func = funcs{2};
fkin_jlims = jlims{2};

%% Compare kinematics
if exist('h_o2a','var')
    delete(h_o2a);
end
h_o2a = triad('Parent',axs,'Matrix',eye(4),'Scale',0.25,'LineWidth',2);

% Define joint configuration
nJoints = size(fkin_jlims,1);
q = 2*pi*rand(nJoints,1) - pi;

% Enforce joint limits
bin_n = q < fkin_jlims(:,1);
q(bin_n) = fkin_jlims(bin_n);
bin_p = q > fkin_jlims(:,2);
q(bin_p) = fkin_jlims(bin_p);

% Double check joint limits
for i = 1:size(q,1)
    if q(i) < fkin_jlims(i,1)
        fprintf(2,'q(%d) below limit: %.6f < %.6f\n',i,q(i),fkin_jlims(i,1));
    end
    if q(i) > fkin_jlims(i,2)
        fprintf(2,'q(%d) above limit: %.6f > %.6f\n',i,q(i),fkin_jlims(i,2));
    end
end

% Set configuration
Q = homeConfiguration(robot);
for i = 1:numel(Q)
    Q(i).JointPosition = q(i);
end
% Show robot in configuration
show(robot,Q,'Parent',axs,'PreservePlot',false);

% Test forward kinematics 
% - From rigid body tree
H_rb2o = getTransform(robot,Q,'tool0');
% - From returned function handle
H_hk2o = fkin_func(q); 
% % - From function created using forward kinematic string
% H_fk2o = fkin_SIA20f(q);

% Visualize results
h_rb2o = triad('Parent',h_o2a,'Matrix',H_rb2o,'Scale',0.25,'LineWidth',2,...
    'AxisLabels',{'x_{rb}','y_{rb}','z_{rb}'});
h_hk2o = triad('Parent',h_o2a,'Matrix',H_hk2o,'Scale',0.35,'LineWidth',2,...
    'AxisLabels',{'x_{hk}','y_{hk}','z_{hk}'});
% h_fk2o = triad('Parent',h_o2a,'Matrix',H_fk2o,'Scale',0.45,'LineWidth',2,...
%     'AxisLabels',{'x_{fk}','y_{fk}','z_{fk}'});

% Hide patch
kids = get(axs,'Children');
set(kids(1:numel(robot.BodyNames)),'Visible','off');

%%
