%%  class URx_ROS 
%   This class creates a matlab object that uses the MATLAB ROS toolbox
%   to interface with a running URx ROS framework. The class functions
%   can issue commands and trajectories and receive data from ROS enabled
%   UR manipulators.
%   
%   Levi DeVries USNA

%   Updates
%       - Helmich:  Updated properties. Method convJoints2ROSJoints not
%       needed? Added inverseKinematics object to improve calculations and
%       hopefully speed.
%
classdef URx_ROS < matlab.mixin.SetGet
    
    properties(GetAccess = 'public', SetAccess = 'public')
        jointTrajectory(6,:) double {mustBeReal,mustBeFinite}
        
    end
    
    properties(GetAccess = 'public', SetAccess = 'private')
        trajPub
        jsSub
        tfSub
        jointNames_ROS = {'elbow_joint','shoulder_lift_joint','shoulder_pan_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'}; % names of SIA20F joints in ROS
        jointOrder_class = {'shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'}; % names of SIA20F joints in ROS
        trajMsg
        homeConfiguration = [0;-pi/2;0;-pi/2;0;0];
        jointAngles(6,1) double {mustBeReal,mustBeFinite}
        jointSpeeds(6,1) double {mustBeReal,mustBeFinite}
        jointState(6,2) double {mustBeReal,mustBeFinite}
        robot
        HTform(4,4) double {mustBeReal,mustBeFinite}
        ikin
    end
    
    methods(Access='public')
        function obj = URx_ROS(name)
            obj.jointTrajectory = [0; -pi/2; 0; -pi/2; 0; 0];   % Home
            obj.trajPub = rospublisher(append(name,'/joint_trajectory_MATLAB'),'IsLatching',false);
            obj.jsSub = rossubscriber(append(name,'/joint_states'),'sensor_msgs/JointState',@obj.parseCallback);
            obj.tfSub = rossubscriber('/tf','tf2_msgs/TFMessage',@obj.tfCallback);
            tmp = receive(obj.jsSub,0.1);
            obj.jointAngles = tmp.Position;
            obj.jointSpeeds = tmp.Velocity;
            
            % Load urdf data
            obj.robot = loadrobot("universalUR10", "DataFormat", "column", "Gravity", [0 0 -9.81]);
            
            % Orient H for ikin calculations later
            obj.HTform = getTransform(obj.robot, obj.jointTrajectory, "tool0");
            obj.HTform = obj.HTform*Rx(-pi/2)*Ry(pi/2);
            
            % Inverse Kinematics Solver With Parameters
            obj.ikin = inverseKinematics('RigidBodyTree', obj.robot);
            obj.ikin.SolverParameters.AllowRandomRestart = false;
        end
        
        function delete(obj)
            rosshutdown
            clear obj.jsSub obj.trajPub
            delete(obj);
        end
        
    end
    
    methods
        % Returns 6x1 current joint angles
        function jointAngles = get.jointAngles(obj) % write as callback instead of direct query
            tmp = receive(obj.jsSub,0.1);
            % We want to see joints in the correct order when using MATLAB
            % to make programming easier. ROS does not see joint angles in
            % same order.
            q = obj.convROSJoints2Joints(tmp.Position);
            jointAngles = q;
        end
        
        % Returns 6x1 current joint speeds
        function jointSpeeds = get.jointSpeeds(obj) % write as callback instead of direct query
            tmp = receive(obj.jsSub,0.1);
            % We want to see joints in the correct order when using MATLAB
            % to make programming easier. ROS does not see joint vels in
            % same order.
            dq = obj.convROSJoints2Joints(tmp.Velocity);
            jointSpeeds = dq;
        end
        
        % Returns 6x1 current joint state (first column is angles and
        % second column is speeds)
        function jointState = get.jointState(obj) % write as callback instead of direct query
            tmp = receive(obj.jsSub,0.1);
            jointState = [tmp.Position, tmp.Velocity];
        end
        
        %
        function sendjointTraj(obj, jointAngles, jointVelocities, timeArray)
            msg = packagePointsList_URx(jointAngles, jointVelocities, timeArray);
            send(obj.trajPub,msg)
        end
        
        %
        function goHome(obj)
            % Create joint state command (need only do once)
            msg = rosmessage(obj.trajPub); % creates message of the correct message type
            msg.JointNames = obj.jointNames_ROS; % append names to message
            msg.Points(1) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            msg.Points(1).Positions = obj.homeConfiguration;
            msg.Points(1).Velocities = zeros(6,1);
            msg.Points(1).TimeFromStart.Sec = 5.0;
            
            send(obj.trajPub,msg)
        end
        
        %
        function msg = send_jointAngles(obj, jAngs, time)
            
            if nargin < 2
                time = 5.0;
            end

            msg = rosmessage(obj.trajPub);
            msg.JointNames = obj.jointNames_ROS;
            msg.Points(1) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            
            msg.Points(1).Positions = jAngs;
            msg.Points(1).Velocities = zeros(6,1);
            msg.Points(1).TimeFromStart.Sec = time;
            
            send(obj.trajPub,msg);
        end
        
        %
        function msg = send_taskPos(obj, tPos, time)

            if nargin < 2
                time = 5.0;
            end

            % tPos must be 3x1
            % Adjust the default H for desired task position
            obj.HTform = getTransform(obj.robot, obj.jointAngles, "tool0");
            % obj.HTform = obj.HTform*Rx(-pi/2)*Ry(pi/2);
            obj.HTform(1:3, 4) = tPos;
            
            [q_sol, q_info] = obj.ikin("tool0", obj.HTform, [0.1, 0.1, 0.1, 10, 0.1, 10],...
                                        obj.jointAngles);
            
            if isequal(q_info.Status, 'success')
                msg = rosmessage(obj.trajPub);
                msg.JointNames = obj.jointNames_ROS;
                msg.Points(1) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                
                msg.Points(1).Positions = q_sol;
                msg.Points(1).Velocities = zeros(6,1);
                msg.Points(1).TimeFromStart.Sec = time;
                
                send(obj.trajPub, msg);
            else
                error("Joint angles couldn't be found. Try again.");
            end           
        end
        
        %
        function msg = send_jTrajectory(obj, path)
            q = path.angles;
            jointAngles_ROS = convJoints2ROSJoints(obj, q);
            jointVelocities = zeros(size(jointAngles_ROS));
            timeFromStart = path.times;
            msg = packagePointsList_URx(jointAngles_ROS, jointVelocities, timeFromStart);
            
            send(obj.trajPub, msg);
        end
        
        %
        function q = convROSJoints2Joints(obj,q_ros)
            % convert order of joint angles from ROS reported
            % sequence to controller reported sequence
            
            sz = size(q_ros);
            ind = find(sz==6);
            
            if(ind==1)
                q = [q_ros(3,:); q_ros(2,:); q_ros(1,:); q_ros(4,:); q_ros(5,:); q_ros(6,:)];
            elseif(ind==2)
                q = [ q_ros(:,3) q_ros(:,2) q_ros(:,1) q_ros(:,4) q_ros(:,5) q_ros(:,6) ];
            end
            
            % flip elements 3 and 1 to give:
            % [shoulder_pan; shoulder_lift; elbow; wrist_1; wrist_2;wrist_3]
            %             q = [q_ros(3); q_ros(2); q_ros(1); q_ros(4); q_ros(5); q_ros(6)];
        end
        
        %
        function q_ros = convJoints2ROSJoints(obj,q)
            % convert order of joint angles from controller reported
            % sequence to ROS reported sequence
            
            % size of input array
            sz = size(q);
            ind_jAngs = find(sz==6); % find whether the matrix is row or column oriented
            
            if(ind_jAngs==1) % 6xn matrix
                q_ros = [q(3,:); q(2,:); q(1,:); q(4,:); q(5,:); q(6,:)];
            elseif(ind_jAngs==2) % nx6 matrix
                q_ros = [ q(:,3) q(:,2) q(:,1) q(:,4) q(:,5) q(:,6) ];
            end
            % flip elements 3 and 1 to give:
            % [elbow, shoulder_lift, shoulder_pan, wrist 1-3]
            %             q_ros = [q(3); q(2); q(1); q(4); q(5); q(6)];
        end
        
        %
        function parseCallback(obj, ~, message)
            jAngs =     convROSJoints2Joints(obj,message.Position);
            jSpeeds =   convROSJoints2Joints(obj,message.Velocity);
            
            obj.jointState = [jAngs jSpeeds];
            obj.jointAngles = jAngs;
            obj.jointSpeeds = jSpeeds;
        end

        %
        function tfCallback(obj,~,message)
            % tfCallback parses information from ros tf to a
            %   homogenous transformation
            %
            %   Output(s)
            %       HTform - 4x4 array element of SE(3) representing the homogenous
            %               transformation (pose) of frame b with respect to frame a.
            %               Pretty sure linear units are specified in meters.
            %
            %   V. Tran, 07Oct2021, USNA
            %   ***Updates***
            %       - figured out that the quaternion parsing has to be [w x y z] for
            %       quat2rotm to work
            %
            %   V. Tran, 27Oct2021, USNA
            
            % Parse tf
            tf = message.Transforms;
            if strcmpi(tf(1).ChildFrameId,'tool0_controller')
                tf_trans = ...
                    1000*[tf.Transform.Translation.X ; tf.Transform.Translation.Y ; tf.Transform.Translation.Z];
                tf_quat = ...
                    quaternion([tf.Transform.Rotation.W  tf.Transform.Rotation.X  tf.Transform.Rotation.Y  tf.Transform.Rotation.Z]);
                % must be in [w x y z] order for quat2rotm to work
                
                % Convert quaternion
                tf_rotm = quat2rotm(tf_quat);
                
                % Combine to form Htf
                obj.HTform = [tf_rotm tf_trans; 0 0 0 1];
            end
        end
        
    end
    
end