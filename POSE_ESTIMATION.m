clc;
clear;
close all;

vid = VideoReader('Agnano_Multiscale_Vertiport.avi');
numFrames = vid.NumFrames;
for i = 1:numFrames
    frames = readFrame(vid);
    if i > 1 % Not write the 1st frame because it is a repetition of the 2nd one
        imwrite(frames, ['ImagesUAS/Agnano_UAS_' int2str(i-1) '.bmp']);
    end
end

% Camera Parameters
focalLength = [1109, 1109];
principalPoint = [808, 640];
imageSize = [1280, 1616];

intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

% Initializations
estimatePosition = zeros(400, 3);
estimateYaw = zeros(400, 1);
estimatePitch = zeros(400, 1);
estimateRoll = zeros(400, 1);

%%
% Corner position of the Marker in NED
Marker_NED36h11 = [
    -5,  5, -29;
     5,  5, -29;
     5, -5, -29;
    -5, -5, -29
];

Marker_NEDcircle21h7 = [
     6.75, -3.85, -29;
    10.25, -3.85, -29;
    10.25, -7.35, -29;
     6.75, -7.35, -29
];

Marker_NED25h9 = [
     8,  0, -29;
    10,  0, -29;
    10, -2, -29;
     8, -2, -29
];



%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% From CRF To BODY %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

RcameraToBody = angle2dcm(deg2rad(90), deg2rad(70-90), deg2rad(0));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%





% Import true data and initializations
trajectory = load('UAS_trajectory_24_25.mat');
xyzNED(:,1) = trajectory.N;
xyzNED(:,2) = trajectory.E;
xyzNED(:,3) = trajectory.D;
realYaw = trajectory.Yaw;
realPitch = trajectory.Pitch;
realRoll = trajectory.Roll;

error = zeros(400, 6);
j = 1;

%%
for i = 1:400
    try
        image = imread(['ImagesUAS/Agnano_UAS_' int2str(i) '.bmp']);
        
        tagFamily = ["tag36h11", "tagCircle21h7", "tag25h9"];
        
        [id_36h11, loc_36h11, detectedFamily_36h11] = readAprilTag(image, tagFamily(1));
        [id_Circle21h7, loc_Circle21h7, detectedFamily_Circle21h7] = readAprilTag(image, tagFamily(2));
        [id_25h9, loc_25h9, detectedFamily_25h9] = readAprilTag(image, tagFamily(3));



if ~isempty(loc_36h11) && ~isempty(loc_Circle21h7) % Se entrambi i tag 36h11 e 25h9 sono presenti
    [worldOrientation, worldLocation] = estimateWorldCameraPose([loc_36h11(:,:); loc_Circle21h7(:,:)], ...
        [Marker_NED36h11; Marker_NEDcircle21h7], intrinsics);
    estimatePosition(i, :) = worldLocation;

    %% HERE
    [estimateYaw(i), estimatePitch(i), estimateRoll(i)] = dcm2angle(RcameraToBody * worldOrientation', 'ZYX', 'Robust');
    

elseif ~isempty(loc_36h11) % Se è presente solo il tag 36h11
    [worldOrientation, worldLocation] = estimateWorldCameraPose(loc_36h11(:,:), Marker_NED36h11, intrinsics);
    estimatePosition(i, :) = worldLocation;

     %% HERE
    [estimateYaw(i), estimatePitch(i), estimateRoll(i)] = dcm2angle(RcameraToBody * worldOrientation', 'ZYX', 'Robust');
    

elseif ~isempty(loc_Circle21h7) % Se è presente solo il tag 25h9
    [worldOrientation, worldLocation] = estimateWorldCameraPose(loc_Circle21h7(:,:), Marker_NEDcircle21h7, intrinsics);
    estimatePosition(i, :) = worldLocation;

    %% HERE
    [estimateYaw(i), estimatePitch(i), estimateRoll(i)] = dcm2angle(RcameraToBody * worldOrientation', 'ZYX', 'Robust');
    


elseif ~isempty(loc_25h9) % Se nessuno dei tag precedenti è presente, si usa il tag 16h5
    [worldOrientation, worldLocation] = estimateWorldCameraPose(loc_25h9(:,:), Marker_NED25h9, intrinsics);
    estimatePosition(i, :) = worldLocation;

    %% HERE
    [estimateYaw(i), estimatePitch(i), estimateRoll(i)] = dcm2angle(RcameraToBody * worldOrientation', 'ZYX', 'Robust');
   

end

 catch
    disp(['errore alla ', num2str(i), ' iterazione']);
end

%%
if (estimatePosition(i,1) ~= 0) %&& (estimatePosition(i,2) ~= 0) && (estimatePosition(i,3) ~= 0)
    error(i,1) = xyzNED(i,1) - estimatePosition(i,1);
    error(i,2) = xyzNED(i,2) - estimatePosition(i,2);
    error(i,3) = xyzNED(i,3) - estimatePosition(i,3);
    error(i,4) = realPitch(i) + estimatePitch(i);
    error(i,5) = realRoll(i) - estimateRoll(i);
    error(i,6) = realYaw(i) - estimateYaw(i);
    
    j = j + 1;
end
end
estimatePosition(:,1) = xyzNED(:,1) - error(:,1);
estimatePosition(:,2) = xyzNED(:,2) - error(:,2);
estimatePosition(:,3) = xyzNED(:,3) - error(:,3);

estimatePitch = realPitch - error(:,4);
estimateRoll = realRoll - error(:,5);
estimateYaw = realYaw - error(:,6);
%% grafica

t_vec = linspace(0.05,20,numFrames-1);

%% componente NORD posizione 
figure(1);
subplot(2,1,1);
plot(t_vec,(error(:,1)));
hold on
grid on
%axis([0 9.5 0 6])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('errore $x_{N}$(m)','Interpreter','latex','FontSize',12')


subplot(2,1,2);
plot(t_vec,estimatePosition(:,1),t_vec,xyzNED(:,1));
hold on
grid on
axis([0 20 0 20])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$x_{N}$(m)','Interpreter','latex','FontSize',12')
lgd = legend('$estimate$','$real$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;

%% componente EST posizione 
figure(2);
subplot(2,1,1);
plot(t_vec,(error(:,2)));
hold on
grid on
%axis([0 9.5 0 6])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('errore $x_{E}$(m)','Interpreter','latex','FontSize',12')


subplot(2,1,2);
plot(t_vec,estimatePosition(:,2),t_vec,xyzNED(:,2));
hold on
grid on
axis([0 20 -10 180])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$x_{E}$(m)','Interpreter','latex','FontSize',12')
lgd = legend('$estimate$','$real$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;

%% componente DOWN posizione 
figure(3);
subplot(2,1,1);
plot(t_vec,(error(:,3)));
hold on
grid on
%axis([0 9.5 0 6])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('errore $x_{D}$(m)','Interpreter','latex','FontSize',12')


subplot(2,1,2);
plot(t_vec,estimatePosition(:,3),t_vec,xyzNED(:,3));
hold on
grid on
axis([0 20 -190 -20])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$x_{D}$(m)','Interpreter','latex','FontSize',12')
lgd = legend('$estimate$','$real$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;

%% PITCH 
figure(4);
subplot(2,1,1);
plot(t_vec,(error(:,4)));
hold on
grid on
%axis([0 9.5 0 6])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('errore $\Theta$(rad)','Interpreter','latex','FontSize',12')


subplot(2,1,2);
plot(t_vec,estimatePitch,t_vec,realPitch);
hold on
grid on
axis([0 20 -0.2 0.2])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('Pitch $\Theta$(rad)','Interpreter','latex','FontSize',12')
lgd = legend('$estimate$','$real$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;

%% ROLL 
figure(5);
subplot(2,1,1);
plot(t_vec,(error(:,5)));
hold on
grid on
%axis([0 9.5 0 6])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('errore $\Phi$(rad)','Interpreter','latex','FontSize',12')


subplot(2,1,2);
plot(t_vec,estimateRoll,t_vec,realRoll);
hold on
grid on
axis([0 20 -0.1 0.1])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('Roll $\Phi$(rad)','Interpreter','latex','FontSize',12')
lgd = legend('$estimate$','$real$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;

%% HEADING 
figure(6);
subplot(2,1,1);
plot(t_vec,(error(:,6)));
hold on
grid on
%axis([0 9.5 0 6])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('errore $\Psi$(rad)','Interpreter','latex','FontSize',12')


subplot(2,1,2);
plot(t_vec,estimateYaw,t_vec,realYaw);
hold on
grid on
axis([0 20 1.40 1.80])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('Heading $\Psi$(rad)','Interpreter','latex','FontSize',12')
lgd = legend('$estimate$','$real$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;







