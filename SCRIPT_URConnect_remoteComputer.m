%% SCRIPT_URConnect_remoteComputer


% if running this script from a windows machine:
% 1) Connect to the "SUAS_JudgesServer" network
% 2) open a command line terminal: start->command prompt
% 3) In the terminal type "ipconfig" and find your IP address. It should be
%    something in the 10.10.130.XXX range
% 4) in the command prompt type "ping 10.10.130.178" you should get a
% response. This is the ROS computer. If you can complete 1-4 you are ready
% to connect to the robot


%% Run system call to find IP address

system('ipconfig')

% Look in command window to find your IP address. It should be something
% like 10.10.130.XXX, e.g. 10.10.130.168

%% Configure IP addresses to connect


% myIP = '10.10.130.124';
% hostIP = '10.10.130.178';
myIP = '10.0.0.1';
hostIP = '10.0.0.2';
myIP = '10.10.130.168';
hostIP = '10.10.130.178';


%% Connect to ROS on host IP
rosinit;
robot = URx_ROS(hostIP);

%% Do whatever you need to do

robot.jointAngles


%% Append angles into array to define a trajectory
numSamps = 5;
angs = zeros(numSamps,6);
for ii = 1:numSamps
    angs(ii,:) = robot.jointAngles;
    pause
end

%% make smoother trajectory out of collected points

figure(1); clf
plot(angs(:,1))
traj(:,1) = linspace(0,angs(1,1),100);

figure(2); clf
plot(angs(:,2))
pf = spline(1:6,angs(:,2));
hold on
plot(linspace(1,6,100),ppval(pf,linspace(1,6,100)))
traj(:,2) = ppval(pf,linspace(1,6,100));

figure(3); clf
plot(angs(:,3))
pf = spline(1:6,angs(:,3));
hold on
plot(linspace(1,6,100),ppval(pf,linspace(1,6,100)))
traj(:,3) = ppval(pf,linspace(1,6,100));

figure(4); clf
plot(angs(:,4))
pf = spline(1:6,angs(:,4));
hold on
plot(linspace(1,6,100),ppval(pf,linspace(1,6,100)))
traj(:,4) = ppval(pf,linspace(1,6,100));

figure(5); clf
plot(angs(:,5))
pf = spline(1:6,[0; angs(:,5);0]);
hold on
plot(linspace(1,6,100),ppval(pf,linspace(1,6,100)))
traj(:,5) = ppval(pf,linspace(1,6,100));

figure(6); clf
plot(angs(:,6))
pf = spline(1:6,[0; angs(:,6);0]);
hold on
plot(linspace(1,6,100),ppval(pf,linspace(1,6,100)))
traj(:,6) = zeros(100,1);

% save('PrintHomeTraj.mat','traj')

%% spline trajectory a different way

oldX = linspace(0,100,100);
newX = linspace(0,100,500);
spl = transpose(spline(oldX, traj', newX));

figure(1); clf
plot(oldX,traj)
hold on
plot(newX,spl,'--');

%% Try move robot along trajectory
angs(2,:) = [0; 0; 0; 0; 0; 0];
angs(3:end,:) = [];
path.angles = angs; % spl';
path.times = linspace(0,6,2);

robot.send_jTrajectory(path);

%% send robot to a specific set of angles
robot.send_jointAngles(zeros(6,1));

%% send robot back home
robot.goHome

%% Shutdown connection when finished
delete(robot)
clear robot



