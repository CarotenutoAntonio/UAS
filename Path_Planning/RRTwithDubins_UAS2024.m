clear all, close all, clc

% % Set RNG seed for repeatable result
rng(1,"twister");

mapData = load("uavMapCityBlock.mat","omap");
omap = mapData.omap;
% Consider unknown spaces to be unoccupied
omap.FreeThreshold = omap.OccupiedThreshold;
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% change map
[xBuilding5,yBuilding5,zBuilding5] = meshgrid(50:100,50:90,0:100);
xyzBuildings2 = [xBuilding5(:) yBuilding5(:) zBuilding5(:)];

obs2 = 0;
updateOccupancy(omap,xyzBuildings2,obs2)


 [xBuilding1,yBuilding1,zBuilding1] = meshgrid(30:50,70:130,70:90);
 [xBuilding2,yBuilding2,zBuilding2] = meshgrid(30:50,60:70,0:100);
 [xBuilding3,yBuilding3,zBuilding3] = meshgrid(30:50,130:140,0:100);
% [xBuilding4,yBuilding4,zBuilding4] = meshgrid(70:80,35:45,0:150);
% 
 xyzBuildings = [xBuilding1(:) yBuilding1(:) zBuilding1(:); %%...
                 xBuilding2(:) yBuilding2(:) zBuilding2(:);...
                 xBuilding3(:) yBuilding3(:) zBuilding3(:)];...
%                 xBuilding4(:) yBuilding4(:) zBuilding4(:)];

obs = 0.65; 
updateOccupancy(omap,xyzBuildings,obs)

show(omap)
%pause;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

startPose = [45 120 95 0];
goalPose = [20 100 60 pi];
figure("Name","StartAndGoal")
hMap = show(omap);
hold on
scatter3(hMap,startPose(1),startPose(2),startPose(3),30,"red","filled")
scatter3(hMap,goalPose(1),goalPose(2),goalPose(3),30,"green","filled")
hold off
view([-31 63])

%pause

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %cambio Va 6 o 9

ss = ExampleHelperUAVStateSpace("MaxRollAngle",pi/6,...    % pi/6
                                "AirSpeed",9,...   %%%%%%  6 
                                "FlightPathAngleLimit",[-0.1 0.1],...% 0.1 
                                "Bounds",[-20 200; -20 220; 10 200; -pi pi]); % [-500 500; -500 500; 10 200; -pi pi]);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

threshold = [(goalPose-0.5)' (goalPose+0.5)'; -pi pi];

setWorkspaceGoalRegion(ss,goalPose,threshold)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sv = validatorOccupancyMap3D(ss,"Map",omap);
sv.ValidationDistance = 0.1;  % 0.1

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

planner = plannerRRT(ss,sv);
planner.MaxConnectionDistance = 50;
planner.GoalBias = 0.10;  
planner.MaxIterations = 1000;
planner.GoalReachedFcn = @(~,x,y)(norm(x(1:3)-y(1:3)) < 5);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[pthObj,solnInfo] = plan(planner,startPose,goalPose);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




if (solnInfo.IsPathFound)
    figure("Name","OriginalPath")
    % Visualize the 3-D map
    show(omap)
    hold on
    scatter3(startPose(1),startPose(2),startPose(3),30,"red","filled")
    scatter3(goalPose(1),goalPose(2),goalPose(3),30,"green","filled")
    
    interpolatedPathObj = copy(pthObj);
    interpolate(interpolatedPathObj,1000)
    
    % Plot the interpolated path based on UAV Dubins connections
    hReference = plot3(interpolatedPathObj.States(:,1), ...
        interpolatedPathObj.States(:,2), ...
        interpolatedPathObj.States(:,3), ...
        "LineWidth",2,"Color","g");
    


    % Plot simulated UAV trajectory based on fixed-wing guidance model
    % Compute total time of flight and add a buffer
    timeToReachGoal = 1.05*pathLength(pthObj)/ss.AirSpeed;
    waypoints = interpolatedPathObj.States;
    [xENU,yENU,zENU] = exampleHelperSimulateUAV(waypoints,ss.AirSpeed,timeToReachGoal);
    hSimulated = plot3(xENU,yENU,zENU,"LineWidth",2,"Color","r");
    legend([hReference,hSimulated],"Reference","Simulated","Location","best")
    hold off
    view([-31 63])
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% smoothed

if (solnInfo.IsPathFound)
    smoothWaypointsObj = exampleHelperUAVPathSmoothing(ss,sv,pthObj);
    
    figure("Name","SmoothedPath")
    % Plot the 3-D map
    show(omap)
    hold on
    scatter3(startPose(1),startPose(2),startPose(3),30,"red","filled")
    scatter3(goalPose(1),goalPose(2),goalPose(3),30,"green","filled")
    
    interpolatedSmoothWaypoints = copy(smoothWaypointsObj);
    interpolate(interpolatedSmoothWaypoints,1000)
    
    % Plot smoothed path based on UAV Dubins connections
    hReference = plot3(interpolatedSmoothWaypoints.States(:,1), ...
        interpolatedSmoothWaypoints.States(:,2), ...
        interpolatedSmoothWaypoints.States(:,3), ...
        "LineWidth",2,"Color","g");
    
    % Plot simulated flight path based on fixed-wing guidance model
    waypoints = interpolatedSmoothWaypoints.States;
    timeToReachGoal = 1.05*pathLength(smoothWaypointsObj)/ss.AirSpeed;
    [xENU,yENU,zENU] = exampleHelperSimulateUAV(waypoints,ss.AirSpeed,timeToReachGoal);
    hSimulated = plot3(xENU,yENU,zENU,"LineWidth",2,"Color","r");
    
    legend([hReference,hSimulated],"SmoothedReference","Simulated","Location","best")
    hold off
    view([-31 63]);
end