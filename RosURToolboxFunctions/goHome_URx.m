%%  goHome_URx
%   Purpose:
%
%   Inputs:
%       - time2home: time in seconds to reach desired q [seconds]
%   Outputs:
%       - out: delta between desired and current joint position [deg]
%
%   L. DeVries 1Feb22

%   Updates:
%       30Aug22 - added time2home input so user can choose speed.    

function out = goHome_URx(time2Home)

HomePub = rospublisher('/joint_trajectory_MATLAB');
HomeSub = rossubscriber('/joint_states');

pause(1)
% Create joint state command (need only do once)
msg = rosmessage(HomePub); % creates message of the correct message type
jointNames = {'elbow_joint','shoulder_lift_joint','shoulder_pan_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'}; % names of SIA20F joints in ROS
msg.JointNames = jointNames; % append names to message
m0 = receive(HomeSub); % grab initial joint states message

msg.Points(1) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
msg.Points(1).Positions = m0.Position;
msg.Points(1).Velocities = zeros(6,1);
% m0.Position

homePos = [0;-pi/2;0;-pi/2;0;0];
msg.Points(2) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
msg.Points(2).Positions = homePos;
msg.Points(2).Velocities = zeros(6,1);
msg.Points(2).TimeFromStart.Sec = time2Home;

send(HomePub,msg)

disp('sent home message. Monitoring completion progress')
while(norm(m0.Position-homePos)>0.01)
    m0 = receive(HomeSub);
    disp(['Position Error: ' num2str(180/pi*norm(m0.Position-homePos)) ' deg.'])
    pause(0.2)
end

out = 180/pi*norm(m0.Position-homePos);
clear HomePub HomeSub

end