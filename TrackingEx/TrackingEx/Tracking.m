clc;
clear;
close all;

load('SimulationData_2024_periodicalV_v2.mat')

%% Grafica Dati Truth and Measured
figure(1)
% Truth
subplot(3,1,1);
plot(Truth.Time,Truth.EgoPos(:,1),Truth.Time,Truth.TargetPos(:,1))
hold on
grid on
axis([0 25 -400 400])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$x_{East}$(m)','Interpreter','latex','FontSize',12')
lgd = legend('$estimate$','$real$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;

subplot(3,1,2);
plot(Truth.Time,Truth.EgoPos(:,2),Truth.Time,Truth.TargetPos(:,2))
hold on
grid on
axis([0 25 30 70])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$x_{North}$(m)','Interpreter','latex','FontSize',12')
lgd = legend('$estimate$','$real$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;

subplot(3,1,3);
plot(Truth.Time,Truth.EgoPos(:,3),Truth.Time,Truth.TargetPos(:,3))
hold on
grid on
axis([0 25 20 50])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$x_{up}$(m)','Interpreter','latex','FontSize',12')
lgd = legend('$estimate$','$real$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;

%Measures

figure(2)
subplot(3,1,1);
plot(Radar.Time,Radar.Range,'Xb','MarkerFaceColor','auto','MarkerSize',3)
hold on
grid on
%axis([0 25 -400 400])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$Range(m)$','Interpreter','latex','FontSize',12')


subplot(3,1,2);
plot(Radar.Time,Radar.Az,'Xb','MarkerFaceColor','r','MarkerSize',3)
hold on
grid on
%axis([0 25 30 70])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$Azimuth(deg)$','Interpreter','latex','FontSize',12')


subplot(3,1,3);
plot(Radar.Time,Radar.El,'Xb','MarkerFaceColor','auto','MarkerSize',3)
hold on
grid on
%axis([0 25 20 50])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$Elevetion(deg)$','Interpreter','latex','FontSize',12')





%% by considering relative position of target with respect to ownship
Delta_x = Truth.TargetPos(:,1) - Truth.EgoPos(:,1);
Delta_y = Truth.TargetPos(:,2) - Truth.EgoPos(:,2);
Delta_z = Truth.TargetPos(:,3) - Truth.EgoPos(:,3);

R_truth = sqrt(Delta_x.^2 + Delta_y.^2 + Delta_z.^2);
Az_truth = atan2(Delta_y, Delta_x);
Az_truth_deg = rad2deg(Az_truth);
El_truth = asin(Delta_z ./ R_truth);
El_truth_deg = rad2deg(El_truth);






%%

% 2) Conversion from SRF to ENU for the radar data.
% 2.1) Computing cartesian measures in SRF
El_radar_SRF = convang(Radar.El, 'deg', 'rad');
Az_radar_SRF = convang(Radar.Az, 'deg', 'rad');
x_SRF = Radar.Range .* cos(El_radar_SRF) .* cos(Az_radar_SRF);
y_SRF = Radar.Range .* cos(El_radar_SRF) .* sin(Az_radar_SRF);
z_SRF = Radar.Range .* sin(El_radar_SRF);

% 2.2) Transforming cartesian measures from SRF to BRF accounting for sensor
% mounting orientation and location.
x_BRF = x_SRF + Radar.MountingLocation(1);
y_BRF = y_SRF + Radar.MountingLocation(2);
z_BRF = z_SRF + Radar.MountingLocation(3);

% 2.3) cartesian measure from BRF to ENU
x_ENU = x_BRF;
y_ENU = y_BRF;
z_ENU = z_BRF;

% 2.4) cartesian component to polar ones
R_radar = sqrt(x_ENU.^2 + y_ENU.^2 + z_ENU.^2);
Az_radar = atan2(y_ENU, x_ENU);
Az_radar_deg = rad2deg(Az_radar);
El_radar = asin(z_ENU ./ R_radar);
El_radar_deg = rad2deg(El_radar);

%%

% Errors
error_R = R_radar - R_truth(1:end-1);

error_AZ = Az_radar - Az_truth(1:end-1);
error_AZ_deg = rad2deg(error_AZ);

error_El = El_radar - El_truth(1:end-1);
error_El_deg = rad2deg(error_El);


%% Plot compairison measure and truth and errors


figure(3)
% Truth
subplot(3,1,1);
plot(Truth.Time,R_truth,'b');
hold on
grid on
plot(Radar.Time,R_radar,'Xr','MarkerFaceColor','auto','MarkerSize',5);
%axis([0 25 -400 400])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$Range$(m)','Interpreter','latex','FontSize',12')
lgd = legend('$Truth$','$Radar$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;

subplot(3,1,2);
plot(Truth.Time,Az_truth_deg,'b');
hold on
grid on
plot(Radar.Time,Az_radar_deg,'Xr','MarkerFaceColor','auto','MarkerSize',5);
%axis([0 25 -400 400])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$Azimuth$(deg)','Interpreter','latex','FontSize',12')
lgd = legend('$Truth$','$Radar$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;

subplot(3,1,3);
plot(Truth.Time,El_truth_deg,'b');
hold on
grid on
plot(Radar.Time,El_radar_deg,'Xr','MarkerFaceColor','auto','MarkerSize',5);
%axis([0 25 -400 400])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$Elevetion$(deg)','Interpreter','latex','FontSize',12')
lgd = legend('$Truth$','$Radar$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;

%%

figure(4)
subplot(3,1,1);
plot(Radar.Time,error_R,'Xb','MarkerFaceColor','auto','MarkerSize',3)
hold on
grid on
%axis([0 25 -400 400])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$Range error(m)$','Interpreter','latex','FontSize',12')


subplot(3,1,2);
plot(Radar.Time,error_AZ_deg ,'Xb','MarkerFaceColor','r','MarkerSize',3)
hold on
grid on
%axis([0 25 30 70])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$Azimuth error(deg)$','Interpreter','latex','FontSize',12')


subplot(3,1,3);
plot(Radar.Time,error_El_deg,'Xb','MarkerFaceColor','auto','MarkerSize',3)
hold on
grid on
%axis([0 25 20 50])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$Elevetion error(deg)$','Interpreter','latex','FontSize',12')


% Variance

error_R_new = error_R(~isnan(error_R))';
var_R = var(error_R_new);
dev_st_R = std(error_R_new);

error_Az_new = error_AZ(~isnan(error_AZ))';
var_Az = var(error_Az_new);
dev_st_Az = std(error_Az_new);

error_El_new = error_El(~isnan(error_El))';
var_El = var(error_El_new);
dev_st_El = std(error_El_new);


%% End accuracy estimation


%% Tracking Problem


% Matrices
% State Transition Matrix

T = 0.01; %s , filter sampling time

% Costant velocity Model
F = [ 1 T
    0 1];

Zero = zeros(2,2);

Phi = [F Zero Zero
      Zero F Zero
      Zero Zero F];

% Process noise matrix Da capire ????
Qi = [T^3/3 T^2/2
      T^2/2 T];

qx = 1; % scale factor
Qx = qx*Qi;

qy = 1000; % scale factor
Qy = qy*Qi;

qz = 1; % scale factor
Qz = qz*Qi;


Q = [Qx Zero Zero
     Zero Qy Zero
     Zero Zero Qz];


% Measurement covariance matrix
R = [var_R  0     0;
     0    var_Az  0;
     0    0  var_El];


% Ientity matrix
I = eye(6);

% State/Covariance Initializations
% state zero x0 and Covariance zero can be initialized by using first radar measure
% initial velocity is assumed zero
sigmas_vel = [0; 0; 0];

idx = find(~isnan(R_radar), 1,'first');
meas = [R_radar(idx); Az_radar(idx); El_radar(idx)];

% Covariance matrix prediction
P = RadEKFStateCov(R, sigmas_vel,meas);

% x=[x, x_dot, y, y_dot, z, z _dot] ^ENU

x0 = zeros(6,1);
x0(1) = R_radar(idx)*cos(Az_radar(idx))*cos(El_radar(idx));
x0(2) = 0;
x0(3) = R_radar(idx)*sin(Az_radar(idx))*cos(El_radar(idx));
x0(4) = 0;
x0(5) = R_radar(idx)*sin(El_radar(idx));
x0(6) = 0;

% It's x_k^ENU
x_k = x0; % state

z = [R_radar; Az_radar; El_radar]; % measure

%%

P_k= P;
P_plot = nan(6,6,2500);
x_plot = nan(2500,6);
for i=idx+1:2500

% State and prediction
x_kp = Phi*x_k;
P_kp = Phi*P_k*Phi'+Q;


% If there is a maeasurement available
if ~isnan(R_radar(i))

% Jacobian of measurement with respect to state
H = RadEKFH(x_k(1),x_k(3),x_k(5),length(x_k));

% Kalman gain
K_kp = (P_kp*H')*(H*P_kp*H'+ R)^-1;

% Covariance correction
P_kpp = (I-K_kp*H)*P_kp;

% Correct Measurement to do the State correction
z_kp = [R_radar(i); Az_radar(i); El_radar(i)];

z_pred = [sqrt(x_kp(1).^2 + x_kp(3).^2 + x_kp(5).^2); ...
          atan2(x_kp(3),  x_kp(1)) ;...
          asin(x_kp(5)./sqrt(x_kp(1).^2+x_kp(3).^2+x_kp(5).^2))];

% Correct State
x_kpp = x_kp + K_kp*(z_kp-z_pred);
x_k = x_kpp;
P_k = P_kpp;
else
x_k = x_kp;
P_k= P_kp;
end
x_plot(i ,: ) = x_k;
P_plot( :,:, i) = P_k;
end


%% ESTIMATED STATE
figure(5)

truth_rel_motion = Truth.TargetPos - Truth.EgoPos ;

subplot(3,1,1);
plot(Truth.Time(1:end-1),x_plot(:,1))%,'Marker','.',MarkerSize=10);
hold on
grid on
plot(Truth.Time,truth_rel_motion(:,1),'r')
%axis([0 25 -400 400])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$x_{East,rel}$','Interpreter','latex','FontSize',12')
lgd = legend('$estimate$','$real$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;

subplot(3,1,2);
plot(Truth.Time(1:end-1),x_plot(:,3))%,'Marker','.',MarkerSize=10);
hold on
grid on
plot(Truth.Time,truth_rel_motion(:,2),'r')
%axis([0 25 30 70])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$x_{North,rel}$','Interpreter','latex','FontSize',12')
lgd = legend('$estimate$','$real$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;

subplot(3,1,3);
plot(Truth.Time(1:end-1),x_plot(:,5))%,'Marker','.',MarkerSize=10);
hold on
grid on
plot(Truth.Time,truth_rel_motion(:,3),'r')
%axis([0 25 20 50])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$x_{Up,rel}$','Interpreter','latex','FontSize',12')
lgd = legend('$estimate$','$real$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;

%% STANDARD DEVIATION
sigma_x = (P_plot(1,1,:)).^0.5;
sigma_y = (P_plot(3,3,:)).^0.5;
sigma_z = (P_plot(5,5,:)).^0.5;

figure(6)

subplot(3,1,1);
plot(Truth.Time(1:end-1),sigma_x(:))%,'Marker','.',MarkerSize=10);
hold on
grid on
%axis([0 25 -400 400])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$\sigma_x$','Interpreter','latex','FontSize',12')
% lgd = legend('$estimate$','$real$');
% lgd.Interpreter = 'latex'; 
% lgd.FontSize = 11;

subplot(3,1,2);
plot(Truth.Time(1:end-1),sigma_y(:))%,'Marker','.',MarkerSize=10);
hold on
grid on
%axis([0 25 30 70])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$\sigma_y$','Interpreter','latex','FontSize',12')
% lgd = legend('$estimate$','$real$');
% lgd.Interpreter = 'latex'; 
% lgd.FontSize = 11;

subplot(3,1,3);
plot(Truth.Time(1:end-1),sigma_z(:))%,'Marker','.',MarkerSize=10);
hold on
grid on
%axis([0 25 20 50])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$\sigma_z$','Interpreter','latex','FontSize',12')
% lgd = legend('$estimate$','$real$');
% lgd.Interpreter = 'latex'; 
% lgd.FontSize = 11;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% tuning qy %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(7)
load('X_N_1000.mat');
load('X_N_100.mat');
load('X_N_1.mat');
load('time.mat');

plot(time,X_N, time, X_N_1,'LineWidth',1);
hold on
grid on
plot(Truth.Time,truth_rel_motion(:,2),'LineStyle','-.','LineWidth',2,'Color','k')
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('$x_{North,rel}$','Interpreter','latex','FontSize',12')
 lgd = legend('$q_z$ = 1000','$q_z$ = 1','$real$');
 lgd.Interpreter = 'latex'; 
 lgd.FontSize = 11;