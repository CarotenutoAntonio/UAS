% UAS Lecture on Tracking for collision avoidance - APPLICATION
% Scenario simulation

clc;
clear;
close all;

% -------------------------------------------------------------------------
%                             USER INPUT
% Directory of this script
mainDir = [cd '/'];
% Flag for saving retrieved data 
isSave  = 0;
% End of simulation (s)
tStop   = 25;
% Update rate (Hz) for simulation
tUpdate = 100;
% Sampling rate (Hz) for radar
tUpdateR = tUpdate;
% Time after which periodical motion starts (s)
tPeriod  = 5;
% target UAV periodical motion - amplitude (m)
amp      = 10;
% target UAV periodical motion - period (s)
deltaT   = 4;

% -------------------------------------------------------------------------
%                               MAIN
% Adding subfunctions folder 
subDir = [mainDir 'MatlabSubfunctions/'];
addpath(genpath(subDir));
%% Tracking scenario definition

% Name of folder containing scenario data
scenName = 'Napoli';
scenFldr = ['scenarioCreation/Scenarios/' scenName '/'];

% Loading scene latitude, longitude (°) data
load([scenFldr 'SceneLatLong.mat']);

% Computing height above WGS84 ellipsoid (m), needed for scene center
% determination 
Height = geoidheight(SceneCenterLatLong(1),SceneCenterLatLong(2));

% Scene center 
latlonCenter = [SceneCenterLatLong(1:2),Height];

% Creating scenario centered at desired location and desired update rate
scene = uavScenario("UpdateRate",tUpdate,"StopTime",tStop,...
    "ReferenceLocation",latlonCenter);

% Adding terrain based on the Global Multi-Resolution Terrain Elevation
% Data (GMTED2010) data set from Matlab
[Xmin,Ymin,~] = geodetic2enu(SceneLimitsLatLong(1,1),SceneLimitsLatLong(1,2),0,SceneCenterLatLong(1),SceneCenterLatLong(2),0,wgs84Ellipsoid);
[Xmax,Ymax,~] = geodetic2enu(SceneLimitsLatLong(4,1),SceneLimitsLatLong(4,2),0,SceneCenterLatLong(1),SceneCenterLatLong(2),0,wgs84Ellipsoid);
sceneXLim = [Xmin Xmax];
sceneYLim = [Ymin Ymax];
scene.addMesh("terrain",{"gmted2010", sceneXLim, sceneYLim},[1 1 1]);

% Adding buildings as downloaded from open street map
scene.addMesh("buildings", {[scenFldr, 'map.osm'],sceneXLim, sceneYLim,"auto"}, [0.94 0.94 0.94]);

% Defining simulation time vector
timeStep = 1/scene.UpdateRate;
timeVect = 0:timeStep:scene.StopTime;

figure;
show3D(scene);
xlim(sceneXLim);
ylim(sceneYLim);

%% UAV platofrms definition
% egoUAV
waypointsA = [-300 50 30; 100 50 30];
timeA      = [0 tStop];
trajA = waypointTrajectory(waypointsA, "TimeOfArrival", timeA, "ReferenceFrame", "ENU","AutoBank",false);
egoUAV = uavPlatform("EGOUAV", scene, "Trajectory", trajA, "ReferenceFrame", "ENU");
updateMesh(egoUAV, "quadrotor", {10}, [0.6350    0.0780    0.1840], eye(4));

% targetUAV 
timeB             = timeVect;
freq              = 2*pi/deltaT;
deltaTsim         = mean(diff(timeVect));
waypointsB        = nan(length(timeVect),3);
waypointsB(:,3)   = 40;
waypointsB(1,1:2) = [300 50];
for i = 2:length(timeVect)
    % constant velocity along East
    waypointsB(i,1) = waypointsB(i-1,1)-amp*deltaTsim;
    % periodical velocity along North after 5 seconds
    if timeVect(i)<=tPeriod
        waypointsB(i,2) = waypointsB(i-1,2);
    else
        waypointsB(i,2) = waypointsB(1,2)+amp*sin(freq*(timeVect(i)-timeVect(1)));
    end
end

trajB = waypointTrajectory(waypointsB, "TimeOfArrival", timeB, "ReferenceFrame", "ENU","AutoBank",false);
targetUAV = uavPlatform("TGTUAV", scene, "Trajectory", trajB, "ReferenceFrame", "ENU");
updateMesh(targetUAV, "quadrotor", {10}, [0 1 0], eye(4));

% Showing UAVs on scene
show3D(scene);
xlim(sceneXLim);
ylim(sceneYLim);

%% Sensors definition
% Mount a radar on the quadrotor.
% Radar operational parameters (update rate must be <= scene update rate)
% For more details: 
% https://it.mathworks.com/help/radar/ref/radardatagenerator-system-object.html
radarSensor = radarDataGenerator("ScanMode","Electronic","SensorIndex",1,...
    "FieldOfView",[6 10],...
    "ElectronicAzimuthLimits",[-30 30],...
    "ElectronicElevationLimits",[0 20],...
    "UpdateRate", tUpdateR,...
    'MountingAngles',[0 0 0],...
    "MountingLocation",[0 0 -0.3],...
    "HasElevation", true,...
    "ElevationResolution", 5,...
    "AzimuthResolution", 2, ...
    "RangeResolution", 4, ...
    "RangeLimits", [0 1000],...
    'CenterFrequency',24.55e9,...
    "Bandwidth",45e6,...
    "TargetReportFormat","Clustered Detections",...
    "DetectionCoordinates","Sensor rectangular",...
    "HasFalseAlarms",false,...
    "FalseAlarmRate",1e-6,...
    "HasNoise",true,...
    "DetectionProbability",0.9);

radar = uavSensor("Radar",egoUAV,helperRadarAdaptor(radarSensor));

%% Start simulation
[ax, plotFrames] = show3D(scene);
xlim(sceneXLim);
ylim(sceneYLim);
hold(ax,'on');
tp = theaterPlot('Parent',ax);
xlabel('East (m)');
ylabel('North (m)');
zlabel('Up (m)');
legend('off');
cp = coveragePlotter(tp,"Alpha",[0.1 0.1]);

% Initializations for data logging
% Time steps counter(s)
ti       = 1;
k        = 0;
egoPos   = nan(length(timeVect),3);
tgtPos   = nan(length(timeVect),3);
egoQuat  = nan(length(timeVect),4);

 % Read platforms orientation and position
egoPose       = read(egoUAV);
tgtPose       = read(targetUAV);
egoPos(ti,:)  = egoPose(1:3);
tgtPos(ti,:)  = tgtPose(1:3);
egoQuat(ti,:) = egoPose(10:13);

% Setting-up scene
setup(scene);

% Setting-up helper to visualize radar scanning
figure;
scanplt = helperScanPatternDisplay(radarSensor);
scanplt.makeOverviewPlot;


while advance(scene)
    disp(ti);

    % Read detections from radar as [range (m), azimuth (°), elevation (°)]
    % in its reference frame, saved in DetsSRF variable
    [isUpdated,time,~,DetsSRF,~] = GetRadarDet(radar,radarSensor,egoPose);
   
    % Logging radar data when retrieved
    if isUpdated
        k = k+1;
        
        savedDet(k,1:3) = DetsSRF;

        if isnan(savedDet(k,1))
            savedDet(k,4)   = nan;  
        else
            savedDet(k,4)   = timeVect(ti);  
        end
        
    end
    
    show3D(scene,'Parent',ax,'FastUpdate',true);

    VisualizeRadar(radarSensor,egoPose,radar,cp,savedDet(k,:));
    drawnow; 

    scanplt.updatePlot(radarSensor.LookAngle);

    % Update sensors 
    updateSensors(scene);

    % Update time steps counter
    ti = ti+1;

    % Read platforms orientation and position
    egoPose       = read(egoUAV);
    tgtPose       = read(targetUAV);
    egoPos(ti,:)  = egoPose(1:3);
    tgtPos(ti,:)  = tgtPose(1:3);
    egoQuat(ti,:) = egoPose(10:13);
end

% Data saving
Truth.EgoPos              = egoPos;
Truth.EgoAttitude         = quat2eul(quaternion(egoQuat));
Truth.Time                = timeVect;
Truth.TargetPos           = tgtPos;

Radar.Range               = savedDet(:,1);
Radar.Az                  = savedDet(:,2);
Radar.El                  = savedDet(:,3);
Radar.Time                = savedDet(:,4);
Radar.SRF2BRFOrientation  = eul2rotm(radar.MountingAngles*pi/180);
Radar.MountingLocation    = radarSensor.MountingLocation;
Radar.RangeAzElResolution = [radar.SensorModel.RangeResolution radar.SensorModel.AzimuthResolution radar.SensorModel.ElevationResolution];
Radar.UpdateRate          = radar.UpdateRate;

if isSave
    save('Data/SimulationData_2024_periodicalV_v2','Truth','Radar');
end

