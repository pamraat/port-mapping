function[dataStore] = motionControl(Robot,maxTime)
% TURNINPLACE: simple example program to use with iRobot Create (or simulator).
% Reads data from sensors, makes the robot turn in place and saves a datalog.
% 
%   dataStore = TURNINPLACE(Robot,maxTime) runs 
% 
%   INPUTStype
%       Robot       Port configurations and robot name (get from running CreatePiInit)
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data

% 
%   NOTE: Assume differential-drive robot whose wheels turn at a constant 
%         rate between sensor readings.
% 
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots
%
% 	Modified: Liran 2023


% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 100;
end

try 
    % When running with the real robot, we need to define the appropriate 
    % ports. This will fail when NOT connected to a physical robot 
    CreatePort=Robot.CreatePort;
catch
    % If not real robot, then we are using the simulator object
    CreatePort = Robot;
end

% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', []);

% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

SetFwdVelAngVelCreate(Robot, 0, 0);
wheel2Center = 0.16;
maxV = 0.5;

% Set angular velocity
fwdVel = 0.3;
angVel = 0;
timrev = -1;
timturn = -1;
tic

hold on

%% Initialization

while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    
    % CONTROL FUNCTION (send robot commands)
    if(dataStore.bump(end,2) == 1 || dataStore.bump(end,3) == 1 || dataStore.bump(end,7) == 1)
        timrev = dataStore.bump(end,1);
    end
    
    if(timrev ~= -1)
        if(dataStore.bump(end,1) < timrev + 0.25/abs(fwdVel))
            fwdVelnew = -fwdVel;
            angVelnew = 0;
        else
            timrev = -1;
            fwdVelnew = 0;
            angVelnew = -1;
            timturn = dataStore.bump(end,1);
        end
    elseif(timturn ~= -1)
        if(dataStore.bump(end,1) < timturn + pi/3)
            fwdVelnew = 0;
            angVelnew = -0.5;
        else
            fwdVelnew = fwdVel;
            angVelnew = angVel;
            timturn = -0.5;
        end
    else
        fwdVelnew = fwdVel;
        angVelnew = angVel;
    end
    [cmdV,cmdW] = limitCmds(fwdVelnew,angVelnew,maxV,wheel2Center);
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0,0);
    else
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    end
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(Robot, 0,0);