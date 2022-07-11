
clc;clear all;
%genFastCC %To speed up the simulation uncomment it

truckParams = struct;
truckParams.L1 = 5;     % Truck body length
truckParams.L2 = 3;    % Trailer length
truckParams.M  = 1;     % Distance between hitch and truck rear axle along truck body +X-axis

truckParams.W1 = 1.75;               % Truck width
truckParams.W2 = 1.75;               % Trailer width
truckParams.Lwheel = 0.5;             % Wheel length
truckParams.Wwheel = 0.4;   

Q = 10; % Weight driving betaDot -> 0
R = 1; % Weight minimizing steering angle command

% Derive geometric steering limits and solve for LQR feedback gains
[alphaLimit, ...        % Max steady-state steering angle before jack-knife
 betaLimit, ...         % Max interior angle before jack-knife
 alphaDesiredEq, ...    % Sampled angles from stable alpha domain
 alphaGain ...          % LQ gain corresponding to desired alpha
] = exampleHelperCalculateLQGains(truckParams,Q,R);

truckParams.AlphaLimit = alphaLimit;
truckParams.BetaLimit = betaLimit;

rFWD = truckParams.L1*1.2;
rREV = rFWD*2;

% Define parameters for the fixed-rate propagator and add them to the
% control structure
controlParams.MaxVelocity = 0.5; % m/s
controlParams.StepSize = .2; % s
controlParams.MaxNumSteps = 25; % Max number of steps per propagation
controlParams = struct(...
    'MaxSteer',alphaLimit, ...          % (rad)
    'Gains',alphaGain, ...              % ()
    'AlphaPoints',alphaDesiredEq, ...   % (rad)
    'ForwardLookahead',rFWD, ...        % (m)
    'ReverseLookahead',rREV, ...        % (m)
    'MaxVelocity', 0.5, ...               % (m/s)
    'StepSize', 0.2, ...                % (s)
    'MaxNumSteps', 25 ...               % ()
    );

distanceHeuristic = exampleHelperCalculateDistanceMetric(truckParams,controlParams);

xyLimits =  [-60 60; -50 50];

% Construct our state-space
stateSpace = exampleHelperTruckStateSpace(xyLimits, truckParams);

Length1='Length'; value1= {30};
Width1='Width'; value2={15};
Theta1='Theta'; value3={0};
XY1='XY';value4=[0 0];
Length='Length'; value5= {14};
Width='Width'; value6={8};
Theta='Theta'; value7={0};
XY='XY';value8=[-40 -40];

obstacles=[struct(Length1,value1,Width1,value2,Theta1,value3,XY1,value4),struct(Length,value5,Width,value6,Theta,value7,XY,value8)];

validator = exampleHelperTruckValidator(stateSpace,obstacles);
propagator = exampleHelperTruckPropagator(validator,controlParams,distanceHeuristic);

start = [-40 45 0 0 0 0];

% Define the goal configuration such that the truck must reverse into
% position.
goal  = [-15 -45 0 -pi -1 nan];

% Display the problem
figure
show(validator)
hold on
configs = [start; goal];
quiver(configs(:,1),configs(:,2),cos(configs(:,3)),sin(configs(:,3)),.1)
goalFcn = @(planner,q,qTgt)exampleHelperGoalReachedFunction(goal,planner,q,qTgt);

% Construct planner
planner = plannerControlRRT(propagator,GoalReachedFcn=goalFcn,MaxNumIteration=30000);
planner.NumGoalExtension = 3;
planner.MaxNumTreeNode = 15000;
planner.GoalBias = .25;

rng(0)
[trajectory,treeInfo] = plan(planner,start,goal)
% exampleHelperPlotTruck(trajectory);
% hold off

% Visualize path and waypoints
% exampleHelperPlotTruck(trajectory);
hold off
