clear
clc

gravity = 9.81;

% Mass and geometric parameters
mass = 13.5; %%kg
Jx   = 0.8244; % Kg*m^2
Jy   = 1.135;
Jz   = 1.759;
Jxz  = 0.1204;
S=0.55; % m^2
b=2.8956; % m
c=0.18994; % m
AR = b^2/S;

% Propeller parameters
S_prop = 0.2027; % m^2
ro = 1.2682; % Kg/m^3
k_motor = 80;
k_t = 0;
k_omega = 0;

%% initial conditions
pn0    = 0;  % initial North position
pe0    = 0; % initial East position
pd0    = 0;% initial Down position (negative altitude)
u0     = 0.01;  % initial velocity along body x-axis
v0     =  0; % initial velocity along body y-axis
w0     =  0; % initial velocity along body z-axis
phi0   =  0; % initial roll angle
theta0 =  0.001; % initial pitch angle
psi0   =  0; % initial yaw angle
p0     =  0; % initial body frame roll rate
q0     =  0;% initial body frame pitch rate
r0     =  0; % initial body frame yaw rate

X0 = [pn0; pe0; pd0; u0; v0; w0; phi0; theta0; psi0; p0; q0; r0];

%% Definitions of Gamma
Gamma = Jx*Jz - Jxz^2;
Gamma1 = Jxz*(Jx - Jy + Jz)/Gamma;
Gamma2 = (Jz*(Jz - Jy) + Jxz^2)/Gamma;
Gamma3 =Jz/Gamma;
Gamma4 =Jxz/Gamma;
Gamma5 =(Jz - Jx)/Jy;
Gamma6 =Jxz/Jy;
Gamma7 =((Jx - Jy)*Jx + Jxz^2)/Gamma;
Gamma8 =Jx/Gamma;

%% Aerodynamic coefficient
% Longitudinal
Cl_0 = 0.28;
Cd_0 = 0.03;
Cm_0 = -0.02338;
Cl_alpha = 3.45;
Cd_alpha = 0.3;
Cm_alpha = -0.38;
Cl_q = 0;
Cd_q = 0;
Cm_q = -3.6;
Cl_delta_e=-0.36;
Cd_delta_e= 0;
Cm_delta_e= -0.5;
C_prop = 1;
M = 50;
alpha_0 = 0.4712;
% epsilon = 0.1592;

% Lateral
Cy_0 = 0;
Cell_0 = 0;
Cn_0 = 0;
Cy_beta = -0.98;
Cell_beta = -0.12;
Cn_beta = 0.25;
Cy_p= 0;
Cell_p = -0.26;
Cn_p = 0.022;
Cy_r = 0;
Cell_r = 0.14;
Cn_r = -0.35;
Cy_delta_a = 0;
Cell_delta_a = 0.08;
Cn_delta_a = 0.06;
Cy_delta_r = -0.17;
Cell_delta_r = 0.105;
Cn_delta_r = -0.032;
e=0.9;

Cp_0 = Gamma3*Cell_0 + Gamma4*Cn_0;
Cp_beta = Gamma3* Cell_beta + Gamma4* Cn_beta;
Cp_p = Gamma3*Cell_p + Gamma4*Cn_p;
Cp_r = Gamma3 * Cell_r + Gamma4*Cn_r;
Cp_delta_a = Gamma3*Cell_delta_a + Gamma4*Cn_delta_a;
Cp_delta_r = Gamma3*Cell_delta_r + Gamma4 * Cn_delta_r;
Cr_0 = Gamma4*Cell_0 + Gamma8*Cn_0;
Cr_beta = Gamma4* Cell_beta + Gamma8*Cn_beta;
Cr_p = Gamma4*Cell_p + Gamma8*Cn_p;
Cr_r = Gamma4 * Cell_r + Gamma8*Cn_r;
Cr_delta_a = Gamma4*Cell_delta_a + Gamma8*Cn_delta_a;
Cr_delta_r = Gamma4*Cell_delta_r + Gamma8 * Cn_delta_r;

Va = 30; % m/s
Vg=Va;

%% Roll loop
a_phi1= -0.5* ro * Va^2*S*b*Cp_p*b/(2*Va);
a_phi2= 0.5 * ro * Va^2*S*b*Cp_delta_a;
delta_a_max = 0.52;

%% modifico e_max %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

zita_phi = 1.1; % kd 0.75
e_phi_max = 0.5; % kp 0.35

% tune to get acceptable step response
kp_roll = delta_a_max/e_phi_max;

wn_roll = sqrt(kp_roll * abs(a_phi2));

% increase until instability, then back off by 20%
kd_roll = (2*zita_phi*wn_roll-a_phi1)/a_phi2;

%% Course loop
W_course = 50; % both kp and ki


wn_course = wn_roll/W_course;

zita_course = 1.9; % kp

% tune to get acceptable step response
kp_course = 2*zita_course*wn_course*Vg/gravity;

% tune to remove steady state error
ki_course = wn_course^2*Vg/gravity;

%% Sideslip loop
a_beta1 = -ro*Va*S*Cy_beta/(2*mass);
a_beta2= ro*Va*S*Cy_delta_r/(2*mass);

e_beta_max = -0.01; % kp
zita_beta = 0.3; % ki

delta_r_max = 0.61;

% tune to get acceptable step response
kp_beta = delta_r_max/e_beta_max ;
wn_beta = (a_beta1 + a_beta2*kp_beta)/(2*zita_beta);

% tune to remove steady state error
ki_beta = wn_beta^2/a_beta2;

%% Pitch Loop
a_teta1 = -(ro*Va^2*c*S)/(2*Jy)*Cm_q*c/(2*Va);
a_teta2 = -(ro*Va^2*c*S)/(2*Jy)*Cm_alpha;
a_teta3 = (ro*Va^2*c*S)/(2*Jy)*Cm_delta_e;

%% cambio e_max %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

delta_e_max = 0.43;

%e1
%e_teta_max = 0.1; % kp

%e2
e_teta_max = 0.005;

%e3
%e_teta_max = 0.5;


kp_pitch = delta_e_max / e_teta_max * sign(a_teta3);
wn_pitch = sqrt(a_teta2+kp_pitch*a_teta3);

% K_pitch_DC = 0.1;
K_pitch_DC = kp_pitch*a_teta3/(a_teta2 + kp_pitch*a_teta3);

%% cambio zita considero e2
%zita 1
%zita_pitch = 0.9; % kd

%zita 2
%zita_pitch = 0.1;

%zita 3
zita_pitch = 0.001;

kd_pitch = (2*zita_pitch * wn_pitch -a_teta1)/a_teta3;

%% Altitude Loop
W_h = 50; % kp and ki

wn_h = wn_pitch/W_h;
ki_h = wn_h^2/(Va*K_pitch_DC);

zita_h = 2.5; % kp

kp_h = 2*zita_h*wn_h/(Va*K_pitch_DC);

%% Airspeed commanded Pitch Loop
W_V2 = 35; % kp and ki

wn_V2 = wn_pitch/W_V2;
ki_v2 = -wn_V2^2/gravity/K_pitch_DC;

zita_V2 = 0.01; % kp

Va_trim = 30;
alpha_trim = 20*3.14/180; %20
delta_e_trim = -0.2; %-0.2
delta_t_trim = 0.5; %1
a_v1 = ro * Va_trim*S /mass * (Cd_0 + Cd_alpha*alpha_trim+Cd_delta_e*delta_e_trim)+ro * S_prop /mass *C_prop*Va_trim;
a_v2 = ro*S_prop/mass * C_prop * k_motor^2*delta_t_trim;
kp_v2 = -(a_v1 - 2*zita_V2 * wn_V2)/gravity/K_pitch_DC;

%% Airspeed commanded throttle Loop
wn_V = 5; % kp and ki
zita_V = 2; % kp

ki_v = wn_V^2 / a_v2;
kp_v = (2*zita_V*wn_V-a_v1)/a_v2;

%% Wind
s = tf('s');

Lu = 300;   % m
Lv = Lu;    % m
Lw = 100;   % m

sigma_u = 2.12;            % 2.12 % 1.06; % m/s
sigma_v = sigma_u;         % 2.12 % 1.06; % m/s
sigma_w = 1.4;             % 1.4% 0.7;  % m/s

H_u = sigma_u*(sqrt(2*Va/Lu))*1/(s+Va/Lu);
H_v = sigma_v*(sqrt(3*Va/Lv))*((s+Va/sqrt(3)*Lv)/(s+Va/Lv)^2);
H_w = sigma_w*(sqrt(3*Va/Lw))*((s+Va/sqrt(3)*Lw)/(s+Va/Lw)^2);

