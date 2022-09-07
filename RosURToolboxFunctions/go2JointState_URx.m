%%  go2JointState_URx
%   
%   Inputs:
%       - time2state: number of seconds to travel to desired q (seconds)
%       - joint_angles: desired angles to send to robot (radians)
%       - namesp: robot's namespace in ROS
%
%   Outputs:
%       - out: delta between desired q and current q in degrees
%
%   L. DeVries 1Feb21
%

function out = go2JointState_URx(time2state, joint_angles, namesp)

narginchk(1,3)
if isequal(namesp, '')
    HomePub = rospublisher('/joint_trajectory_MATLAB');
    HomeSub = rossubscriber('/joint_states');
else
    HomePub = rospublisher(append('/', namesp, '/joint_trajectory_MATLAB'));
    HomeSub = rossubscriber(append('/', namesp, '/joint_states'));
end

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

msg.Points(2) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
msg.Points(2).Positions = joint_angles;
msg.Points(2).Velocities = zeros(6,1);
msg.Points(2).TimeFromStart.Sec = time2state;

send(HomePub,msg)

disp('sent home message. Monitoring completion progress')
while(norm(m0.Position-joint_angles)>0.01)
    m0 = receive(HomeSub);
    disp(['Position Error: ' num2str(180/pi*norm(m0.Position-joint_angles)) ' deg.'])
    pause(0.2)
end

out = 180/pi*norm(m0.Position-joint_angles);
clear HomePub HomeSub

end