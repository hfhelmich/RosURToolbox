%%  collectURSim.m
%   Running UR10 simulation and saving data. Sections of code here are
%   useful when saving data in the future.
%
%   M. Kutzer 30Aug22

%
%

%% Initialization Code
ur = URsim;
ur.Initialize

%% Collect home
ur.Home;
pause(1);
sim_jointsHome = ur.Joints;
sim_poseHome = ur.Pose;

%% Collecting simulation data from ros dataset
dstr = input('What date is the data for (in yyyymmdd format)?');
load('UR10Info_%s.mat',dstr);

sim_joints = robot_joints;
sim_pose = {};
% Moving sim to joint angles and then collecting poses
for i = 1:size(robot_joints,2)
    ur.Joints = robot_joints(:,i)
    pause(1);
    sim_pose{i} = zeroFPError(ur.Pose);
end

%% Saving simulation data
dstr = datestr(now,'yyyymmdd');
fnameRobotInfo = sprintf('SimUR10Info_%s.mat',dstr);
save(fnameRobotInfo,'sim_joints','sim_jointsHome','sim_pose','sim_poseHome');

%% Code to remember
% ur2 = URsim;
% ur2.Initialize
% ur.Home
% ur.Pose
% zeroFPError(ans)
% ur.Home
% ur.Pose
% ur.Joints
% help ur.Joints
% zeroFPError(ans)
% ur.Pose
% zeroFPError(ans)