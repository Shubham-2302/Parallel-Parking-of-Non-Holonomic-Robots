clc, clear all

%Obstacle Map Generations

image = imread("Obstacle_Map.jpg");
grayimage = rgb2gray(image);
bwimage = grayimage < 200;
map= binaryOccupancyMap(bwimage);

%show(map)

costmap = vehicleCostmap(map);
%plot(costmap)

validator = validatorOccupancyMap;
validator.Map = map;
planner = plannerHybridAStar(validator,'MinTurningRadius',4,'MotionPrimitiveLength',6);
startPose = [5 60 0]; % [meters, meters, radians]
goalPose = [38 8 pi/4];
final_path = plan(planner,startPose,goalPose);

%show(planner)

%Initialising an Matlab Controller to follow the path

controller = controllerPurePursuit;
controller.Waypoints = final_path.States(:,1:2);
controller.DesiredLinearVelocity = 6;
controller.MaxAngularVelocity = 1;
controller.LookaheadDistance = 1.5;

robotInitialLocation = final_path.States(1,1:2);
robotGoal = final_path.States(end,1:2);
initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation]';
distanceToGoal = norm(robotInitialLocation - robotGoal);
goalRadius = 0.1;

robot = differentialDriveKinematics("TrackWidth", 3, "VehicleInputs", "VehicleSpeedHeadingRate");

% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.4;

while( distanceToGoal - goalRadius > 10e-3)
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose(1:3));
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    show(map);
    hold all

    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(final_path.States(:,1), final_path.States(:,2),"k--d")
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 80])
    ylim([0 80])
    
    waitfor(vizRate);
end