%%  SCRIPT_UR3e_executeRose3
%   Create a 3 piece rose trajectory, compile into ROS message, and send
%   to the UR3e robot.
% 
%   Harrison Helmich; 15 July 2022
%

%% Set Up
close;
rosinit;

robotIPa = '10.0.0.2';               %
ura = URx_ROS(robotIPa, '/ura');     % create robot object for specific topics
urMod = 'UR3e';
H_e2o = [   1 0 0 0;
            0 1 0 0;
            0 0 1 0;
            0 0 0 1];

%% Simulation
sim = URsim;
sim.Initialize('UR3');
hold on;

%%  Create trajectory in Task Space
r = 75;    % Radius will scale the size of the rose
% Position 
% Y position N*ones(size(s)) where N determines distance of XZ plane that's
% safe enough for UR3e to operate
f = @(s)[r*(2*sin(3*s)*cos(s)); 600*ones(size(s)); r*2*sin(3*s)*sin(s) + 500];
% Velocity
df = @(s)[r*(2*sin(3*s) + 2*cos(4*s)); zeros(size(s)); r*(4*sin(4*s) - 2*sin(2*s))];
% Acceleration
ddf = @(s)[r*(-4*sin(2*s) + 4*sin(4*s)); zeros(size(s)); r*(-4*cos(2*s) - 4*cos(4*s))];


% % Position 
% % Y position N*ones(size(s)) where N determines distance of XZ plane that's
% % safe enough for UR3e to operate
% f = @(s)[   r*(2*sin(3*s)*cos(s)) + 300;    r*2*sin(3*s)*sin(s) + 300;      400*ones(size(s))];
% % Velocity
% df = @(s)[  r*(2*sin(3*s) + 2*cos(4*s));    r*(4*sin(4*s) - 2*sin(2*s));    zeros(size(s))];
% % Acceleration
% ddf = @(s)[ r*(-4*sin(2*s) + 4*sin(4*s));   r*(-4*cos(2*s) - 4*cos(4*s));   zeros(size(s))];


n = 500;
tF = 10;                   % seconds
t = linspace(0,tF, n);
theta = pi/tF*t;

% Draw desired path in task space
X = zeros(3, n);
Xd = zeros(3, n);
Xdd = zeros(3, n);

for i = 1:n
    s = theta(i);
    % evaluate each pp func at s
    X(:,i)   = f(s);
    Xd(:,i)  = df(s);
    Xdd(:,i) = ddf(s);
end

% Plot task trajectory in 3D
% Shows the shape of the desired path
figure;
plot3(X(1,:),X(2,:),X(3,:)); % x y z
grid on;

q_eu = zeros(6,n);
q_ed = zeros(6,n);
q    = zeros(6,n);
for i = 1:n
    
    H_e2o(1:3,4) = X(:,i);
    % q(:,i) = UR_ikin(urMod, H_e2o)
    [q_eu(:,i), q_ed(:,i)] = ikinUR3e(X(:,i));
end

q_eu = real(q_eu);
q_ed = real(q_ed);

q = q_eu;
qd = zeros(size(q));

path.angles = q;
path.times = t;

pause(1);
close;

%% Run Simulation

for i = 1:5 
    sim.Joints = q(:,100*i);
    pause(1);
end


%%  Run the trajectory

%   With no safety enabled on the matlab trajectory wrapper, it's crucial
%   to send the robot to the first joint angle before starting the
%   trajectory.
ura.send_jointAngles(q(:,1));

%   The path data structure should have a "angles" and "times" data field
%   of congruent sizes to run the below function.
ura.send_jTrajectory(path);

% sendjointTraj(ura, q, qd, t);

%%
ura.send_jointAngles(zeros(6,1));





