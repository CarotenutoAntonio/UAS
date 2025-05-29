clc;
clear;
close all;

% %% t
% t = out.tout(:);
% save ('tz3.mat','t');
% 
% %% h
% h=out.h(:);
% save('hz3.mat','h');
% 
% comm_h=out.comm_h(:);
% save('comm_hz3.mat','comm_h');
% 
% %% Va
% 
% Va=out.Va(:);
% save('Vze3.mat','Va');
% 
% comm_Va=out.comm_Va(:);
% save('comm_Vze3.mat','comm_Va');
% 
% %% course
% course=out.course(:);
% save('coursze3.mat','course');
% 
% comm_course=out.comm_h;
% save('comm_coursze3.mat','comm_course');
% 
% 


%% e1
%% t
te1 = load("te1.mat");

%% h
he1 = load("he1.mat");

comm_he1 = load('comm_he1.mat');

%% Va

Vae1 = load('Vae1.mat');

comm_Vae1 = load('comm_Vae1.mat');

%% course
coursee1 =load('coursee1.mat');

comm_course=load('comm_coursee1.mat');


plot(te1.t,he1.h,te1.t,comm_he1.comm_h);




%% e2
%% t
te2 = load("te2.mat");

%% h
he2 = load("he2.mat");

comm_he2 = load('comm_he2.mat');

%% Va

Vae2 = load('Vae2.mat');

comm_Vae2 = load('comm_Vae2.mat');

%% course
coursee2 =load('coursee2.mat');

comm_coursee2=load('comm_coursee2.mat');

%% e3
%% t
te3 = load("te3.mat");

%% h
he3 = load("he3.mat");

comm_he3 = load('comm_he3.mat');

%% Va

Vae3 = load('Vae3.mat');

comm_Vae3 = load('comm_Vae3.mat');

%% course
coursee3 =load('coursee3.mat');

comm_coursee3=load('comm_coursee3.mat');


%% zita2
%% t
tz2 = load("tz2.mat");

%% h
hz2 = load("hz2.mat");

comm_hz2 = load('comm_hz2.mat');

%% Va

Vze2 = load('Vze2.mat');

comm_Vze2 = load('comm_Vze2.mat');

%% course
coursze2 =load('coursze2.mat');

comm_coursze2=load('comm_coursze2.mat');

%% zita3
%% t
tz3 = load("tz3.mat");

%% h
hz3 = load("hz3.mat");

comm_hz3 = load('comm_hz3.mat');

%% Va

Vze3 = load('Vze3.mat');

comm_Vze3 = load('comm_Vze3.mat');

%% course
coursze3 =load('coursze3.mat');

comm_coursze3=load('comm_coursze3.mat');




figure(1)
plot(te1.t,he1.h,te2.t,he2.h,te3.t,he3.h, te2.t,comm_he2.comm_h)
hold on
grid on
axis([0 30 40 130])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('H(m)','Interpreter','latex','FontSize',12')
lgd = legend('$e_{max}$ = 0.1','$e_{max}$ = 0.005','$e_{max}$ = 0.5','$H_{com}$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;


figure(2)
comm_Va = comm_Vae1.comm_Va*ones(1,length(te1.t));
plot(te1.t,Vae1.Va, te2.t,Vae2.Va, te3.t,Vae3.Va, te1.t,comm_Va)
hold on
grid on
axis([0 30 15 35])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('Va(m/s)','Interpreter','latex','FontSize',12')
lgd = legend('$e_{max}$ = 0.1','$e_{max}$ = 0.005','$e_{max}$ = 0.5','$V_{a,com}$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;

figure(3)
plot(te2.t,he2.h,tz2.t,hz2.h,tz3.t,hz3.h, te2.t,comm_he2.comm_h);
hold on
grid on
axis([0 30 40 130])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('H(m)','Interpreter','latex','FontSize',12')
lgd = legend('$\zeta_1$ = 0.9','$\zeta_2$ = 0.1','$\zeta_3$ = 0.001','$H_{com}$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;

figure(4)
plot(te2.t,Vae2.Va, tz2.t,Vze2.Va, tz3.t,Vze3.Va, te1.t,comm_Va)
hold on
grid on
axis([0 30 15 35])
xlabel('t(s)','Interpreter','latex','FontSize',12);
ylabel('Va(m/s)','Interpreter','latex','FontSize',12')
lgd = legend('$\zeta_1$ = 0.9','$\zeta_2$ = 0.1','$\zeta_3$ = 0.001','$V_{a,com}$');
lgd.Interpreter = 'latex'; 
lgd.FontSize = 11;

