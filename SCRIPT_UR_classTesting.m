%% Create UR robot class interface

robot = URx_ROS('10.0.0.2');

%% Query joint angles

angs = robot.jointAngles

% DO NOT DELETE: angles corresponding to "home" for a conformal print
printHome = [1.7715 ; -2.0385 ; -0.6863; -2.4946 ; 4.5640 ; 0]

% To get constant feed of joint angles
% rostopic echo /joint_states

%% creat a joint trajectory


dt = 1/40;
tF = 10;

i3 = 0;
f3 = pi/4;
d3dt = (f3-i3)/dt;

timeArray = 0:6:1;

jointAngles = zeros(6,length(timeArray));
jointAngles(2,:) = -pi/2;
jointAngles(4,:) = -pi/2;
jointAngles(3,:) = pi/3*sin(2*pi/5*timeArray);
jointAngles(1,:) = 30*pi/180*sin(2*pi/5*timeArray);


jointVelocities = zeros(size(jointAngles));
% jointVelocities(3,:) = 0;
% jointVelocities(3,end) = 0;

%% vic test
timeArray = [0:1:5]'

jointAngles = zeros(6,length(timeArray));
jointAngles = sim_joints(:,1);
% jointAngles(2,:) = -pi/2;
% jointAngles(4,:) = -pi/2;
% jointAngles(3,:) = pi/3*sin(2*pi/5*timeArray);
% jointAngles(1,:) = 30*pi/180*sin(2*pi/5*timeArray);


jointVelocities = ones(size(jointAngles));
% jointVelocities(3,:) = 1;
% jointVelocities(3,end) = 1;


%%
sendjointTraj(robot,jointAngles,jointVelocities,timeArray)