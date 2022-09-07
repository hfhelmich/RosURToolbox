function msg = packagePointsList_URx(jointAngles,jointVelocities,timeFromStart)
% msg = packagePointsList(jointAngles,jointVelocities,timeFromStart) takes
% arrays of joint angles, joint velocities, and the times at which each
% point should be achieved and packages them into a ROS message to be sent
% to the URx (x=3,5,10) robot (or any UR manipulator with 6-axes).
% 
% Inputs:
%   jointAngles: 6xn matrix comprised of the 6 desired joint angles for n
%               desired configurations
%   jointVelocities: 6xn matrix comprised of the 6 desired joint velocities
%               at each of the 6 desired configurations
%   timeFromStart: 1xn vector of times at which each configuration should
%   be achieved.
%
%
% NOTES:
%  - Requires ROS Toolbox
%  - Joint positions, velocities, and timing must be consistent with a
%    continuous trajectory or the Yaskawa FS100 controller will reject with
%    trajectory without providing error feedback or other notice.
%
% L. DeVries, Ph.D.
% WRC USNA, 30 Sept 2021

% check to ensure joint position and velocity matrices are of equal size
if ~isequal(size(jointAngles), size(jointVelocities))
    error('Joint angle and joint velocity matrices must be of equal dimensions')
end
szJ = size(jointAngles);
szV = size(jointVelocities);

% extract the dimension (row/column) in which specifies each joint
dimJ = szJ==6;
dimV = szV==6;
% check that 6 joint angles have been prescribed for the Yaskawa arm
if sum([dimJ dimV])==0
    error('Insufficient number of joint angles provided. Arm has 7 joints')
end

% extract number of time points
n = szJ(~dimJ); % number of time steps/points in trajectory
if n~=length(timeFromStart)
    error('timeFromStart must contain as many time points as position and velocity trajectories')
end

% Create message of the correct message type for the trajectory publisher
msg = rosmessage('trajectory_msgs/JointTrajectory'); 

% joint names of the Yaskawa SIA20F manipulator
jointNames = {'elbow_joint','shoulder_lift_joint','shoulder_pan_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'}; % names of SIA20F joints in ROS

% append joint names to message
msg.JointNames = jointNames; 

% pre-allocate message structure to accommodate all the time points
% specified by the input
msg.Points = arrayfun(@(~) rosmessage('trajectory_msgs/JointTrajectoryPoint'), zeros(n,1));

% iterate through all points in the trajetory filling in positions,
% velocities and times to appropriate fields
for ii = 1:1:n
    msg.Points(ii) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    msg.Points(ii).Positions        = jointAngles(:,ii); 
    msg.Points(ii).Velocities       = jointVelocities(:,ii);
    msg.Points(ii).TimeFromStart    = rosduration(timeFromStart(1,ii));
end

end