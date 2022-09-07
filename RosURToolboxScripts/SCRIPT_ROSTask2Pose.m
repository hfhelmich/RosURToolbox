%% SCRIPT_ROSTask2Pose
%
%   M. Kutzer, 28Oct2021, USNA
clear all
close all
clc

%% Load data set
load('robot_data.mat');

%% Hardcode the values from the "Controller Values of Robot"
X_cont = [...
    -249.38,   20.17,  913.14,  3.1322,  1.2049, -2.0745;... % IMG_3566.HEIC
    -268.76,   42.89,  928.34,  2.2960,  1.1513, -2.4434;... % IMG_3567.HEIC
    -386.64,   23.46,  728.74,  1.1552, -1.4120,  3.5957;... % IMG_3568.HEIC
    -257.76,  200.34,  866.04,  1.0600,  1.1787, -2.1278;... % IMG_3569.HEIC
    -139.77,  177.50,  633.06,  2.6669,  1.5692, -2.9054].'; % IMG_3570.HEIC

q_cont = [...
     -85.07, -144.92, 106.48, -110.73, -26.13,   1.83;... % IMG_3566.HEIC
     -79.30, -138.92, 106.48, -110.73,  -0.74,  10.01;... % IMG_3567.HEIC
     -27.54, -110.56, 122.57, -150.11,  93.71,  -7.82;... % IMG_3568.HEIC
     -79.00, -119.06, 112.32, -150.12,  56.53,  45.95;... % IMG_3569.HEIC
    -149.64, -155.71, 143.34, -150.12,  49.52,   6.01].'; % IMG_3570.HEIC
    
%% Create UR10 visualization(s)
for i = 1:2
    sim(i) = URsim;
    sim(i).Initialize('UR10');
    for j = 1:6
        fld = sprintf('hFrame%d',j);
        hideTriad(sim(i).(fld));
    end
    hideTriad(sim(i).hFrameT)
end
% Consolidate two arms into one axes
sim(2).Axes = sim(1).Axes;

%% Change color of two arms
for j = 0:6
    fld = sprintf('pLink%d',j);
    set( sim(1).(fld), 'FaceColor','r','FaceAlpha',0.6 );
    set( sim(2).(fld), 'FaceColor','g','FaceAlpha',0.6 );
end

%% Compare joint configurations
for i = 1:size(q_cont,2)
    % sim(1).Joints = contJoints2rosJoints( q_cont(:,i) );
    sim(1).Joints = deg2rad(q_cont(:,i));
    % sim(2).Joints = correctROSJoints( robot_joints(:,i) );
    sim(2).Joints = robot_joints_corrected(:,i);
    drawnow;
    pause;
end

%% Calculate pose using forward kinematics & compare to Tran values
% There is an issue with *robot_pose* --> not anymore?

% Using the q from my data collection, calculating the pose using fkin
% and comparing that to my pose data collection

%for i = 1:size(robot_joints,2)
for i = 1:size(robot_joints_corrected,2)
    %q = correctROSJoints( robot_joints(:,i) );
    q = robot_joints_corrected(:,i);
    H_e2o_q = UR_fkin('UR10',q);
    %H_e2o_T = robot_pose{i};
    H_e2o_T = robot_pose_converted{i};
    
    delta_H = H_e2o_q * invSE(H_e2o_T);
    fprintf('H_e2o_q{%d}*(H_e2o_T{%d})^-1 = \n',i,i);
    disp(delta_H);
end

%% Compare ROS joint configurations to "Controller Values"
for i = 1:size(q_cont,2)
    % q = correctROSJoints( robot_joints(:,i) );
    q = robot_joints_corrected(:,i);
    delta_q(:,i) = deg2rad(q_cont(:,i)) - q;
end

fprintf('\nDifference between ROS joint configuration and controller joint configuration (degrees)\n');
for j = 1:size(delta_q,1)
    fprintf('q_%d: [',j);
    fprintf('%7.2f ',rad2deg( delta_q(j,:) ));
    fprintf(']\n');
end

%% Compare "Controller Task" to "Controller Joint" using forward kinematics
% Comparing to controller task variables to fkin of controller joint
% variables.
for i = 1:size(q_cont,2)
    %q = contJoints2rosJoints( q_cont(:,i) );
    q = deg2rad(q_cont(:,i)); 
    H_e2o_cq = UR_fkin('UR10',q);
    
    R_e2o_cT = rotationVectorToMatrix(-X_cont(4:6,i));
    d_e2o_cT = X_cont(1:3,i);
    H_e2o_cT = eye(4);
    H_e2o_cT(1:3,1:3) = R_e2o_cT;
    H_e2o_cT(1:3,4) = d_e2o_cT;
    
    delta_H = H_e2o_cq*invSE(H_e2o_cT);
    fprintf('H_e2o_cq{%d}*(H_e2o_cT{%d})^-1 = \n',i,i);
    disp(delta_H);
end
