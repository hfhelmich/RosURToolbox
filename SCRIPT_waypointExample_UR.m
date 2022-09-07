%% Drive UR10 hardware or ROS Simulation from MATLAB using ROS Toolbox
% SCRIPT_move_SIA20_MATLAB.m demonstrates how to connect to a remote ROS
% Master at a known IP address then subscribe and publish to topics on the
% ROS network. Additional code is included to plot joint states of the
% robot in real time.
%
% Requirements:
%   ROS Toolbox
%
% L. DeVries, USNA, WRC, 6/15/2021

clear all
clc
%% Initialize ROS with remote host (only do once)
hostIP = '10.0.0.2'; % replace with ROS Master IP or localhost depending on setup configuration
% hostPort = '11311'; % host port, not certain this is the default
% setenv('ROS_MASTER_URI',['http://' hostIP ':' hostPort]); % set ROS master uri
% myIP = '10.0.0.118';
% setenv('ROS_IP',myIP); % my own IP address
% setenv('ROS_HOSTNAME',hostIP)

% rosinit(hostIP);
rosinit

%% Create publisher to command joint states (only do once)
pub = rospublisher('/ura/joint_trajectory_MATLAB','IsLatching',false);

%% Create a subscriber to listen to joint states topic (do only once)
% Third input argument is optional and specifies a callback function to
% automatically handle data as it is published to the topic. See
% JointState_Callback.m for more information
%js_sub = rossubscriber('/joint_states','sensor_msgs/JointState',@JS_Callback);
js_sub = rossubscriber('/ura/joint_states','sensor_msgs/JointState'); % Syntax when not using callback

%% Create joint state command (need only do once)
msg = rosmessage(pub); % creates message of the correct message type
jointNames = {'elbow_joint','shoulder_lift_joint','shoulder_pan_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'}; % names of SIA20F joints in ROS
msg.JointNames = jointNames; % append names to message

homePos = [0;-pi/2;0;-pi/2;0;0];


m0 = receive(js_sub);
msg.Points(1) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
msg.Points(1).Positions = m0.Position;
msg.Points(1).Velocities = zeros(6,1);
msg.Points(1).TimeFromStart.Sec = 0.0;



msg.Points(2) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
msg.Points(2).Positions = homePos;
msg.Points(2).Velocities = zeros(6,1);
msg.Points(2).TimeFromStart.Sec = 4.0;

msg.Points(3) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
msg.Points(3).Positions = homePos+[45;0;0*30;0*10;0;0]*pi/180;
msg.Points(3).Velocities = zeros(6,1);
msg.Points(3).TimeFromStart.Sec = 7.0;


% msg.Points(4) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
% msg.Points(4).Positions = homePos+[70;0;40;0;0;30]*pi/180;
% msg.Points(4).Velocities = zeros(6,1);
% msg.Points(4).TimeFromStart.Sec = 10.0;
% 
% msg.Points(5) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
% msg.Points(5).Positions = homePos;
% msg.Points(5).Velocities = zeros(6,1);
% msg.Points(5).TimeFromStart.Sec = 13.0;

% msg.Points(3) = rosmessage('trajectory_msgs/JointTrajectoryPoint')
% msg.Points(3).Positions = [0.75; -0.75; 0.0; 0.8; 0.75; 0.75; 0.0]
% msg.Points(3).Velocities = zeros(7,1);
% msg.Points(3).TimeFromStart.Sec = 5.0



%%
% publish the command
send(pub,msg)

%% close ros connection
rosshutdown
