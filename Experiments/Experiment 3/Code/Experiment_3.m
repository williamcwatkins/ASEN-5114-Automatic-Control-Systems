%% Experiment 3
% William Watkins
% 3 February 2022

%% Clean up
clear all
close all
clc

%% Constants/Properties
Jconv = 1/0.00034171718982094; %[g*cm^2 / lb*in^2]
Jc = 1*10^-7; % [kg*m^2 / g*cm^2]
Rm = 19.2;      % [Ohms]
Lm = 0.0019;    % [Henrys]
Ktau = 40.1*10^-3;  % [Nm/A]
Kb = 1/238 / (2*pi/60); % [V/(rad/s)]
rBig = 2.51; % [in]
rSmall = 0.79; % [in]
Ngearhead = 10;
Ngears = rBig/rSmall;
Jm = 12.5 * Jc; % [kg*m^2]
Jgearhead = 0.6 * Jc; % [kg*m^2]
N = Ngearhead * Ngears;

%% Computing Moments
Jtm = Jm; % [kg*m^2]
rhoAl = 0.097; % [lb/in^2]
h = 0.199; % [in]
wBeam = 0.504; % [in]
dBeam = 10; % [in]
sSquare = 1.75; % [in]
Vsq = sSquare^2 * h; % [in^3]
Vbeam = h * wBeam * dBeam; % [in^3]
mSquare = Vsq * rhoAl; % [lb]
mBeam = Vbeam * rhoAl; % [lb]
Jbeam = (1/12) * mBeam * (dBeam^2 + wBeam^2); % [lb*in^2]
Jsquare = (1/12) * mSquare * 2 * sSquare^2; % [lb*in^2]
z = dBeam/2 + sSquare/2; % [in]
JPA = Jbeam + mBeam*z^2; % [lb*in^2]
Jl = JPA + Jsquare; % [lb*in^2]
Jl = Jl * Jconv * Jc; % [kg*m^2]
Jeq = Jl + N^2 * Jtm; % [kg*m^2]

%% System Coefficients
s3 = (Jeq/(N*Ktau))*Lm;
s2 = (Jeq/(N*Ktau))*Rm;
s1 = N*Kb;
Gd_o = -0.1;
Gp_o = -10;

%% Problem 5

Gp = -0.1 * Gp_o;
Gd = 0;

p = [Jeq*Lm/(N*Ktau) (Jeq*Rm/(N*Ktau)) N*Kb-Gd -Gp];
r = roots(p);

figure('Position', [200 200 1000 800]);
hold on;
grid on;
plot(r, [0 0 0], '*','color','magenta');
xlabel('Re(s)');
ylabel('Im(s)');
title('Poles of System with +G_P')
xlim([-11000 1000])

%% Problem 6

Gp = 0;
Gd = -10*Gd_o;

p = [Jeq*Lm/(N*Ktau) (Jeq*Rm/(N*Ktau)) N*Kb-Gd -Gp];
r = roots(p);

figure('Position', [200 200 1000 800]);
hold on;
grid on;
plot(r, [0 0 0], '*','color','magenta');
xlabel('Re(s)');
ylabel('Im(s)');
title('Poles of System with +G_D')
xlim([-11000 1000])

%% Problem 7

load('Data/StepResponseP5');
TimeP7 = [(1:1:999)' / 1000; ((StepResponseP5(675:675+499,1) - ...
    StepResponseP5(675,1)) / 1000) + 1];
OutputP7 = [zeros(999,1); StepResponseP5(675:675+499,2)-...
    StepResponseP5(675,2)];

% From Experiment 1
Gp = -10; % [V/rad]
Gd = -0.1; % [V/(rad/s)]

simOut5 = sim('PD_Control_Step.slx');
thetaL5(1,:) = simOut5.yout{1}.Values.Data';
tSpan5(1,:) = simOut5.yout{1}.Values.Time';
thetaL5(2,:) = simOut5.yout{2}.Values.Data';
tSpan5(2,:) = simOut5.yout{2}.Values.Time';
figure('Position', [200 200 1000 800]);
hold on;
grid on;
plot(tSpan5(1,:),thetaL5(1,:))
plot(TimeP7, OutputP7,'r-')
plot(tSpan5(2,:),thetaL5(2,:))
legend({'Simulated \theta_L [rad]','Experimental \theta_L [rad]',...
    'Reference Input [rad]'},'Location','northeast')
title('Response to Step Input')
xlabel('Time [s]')
ylim([0 0.8])

%% Problem 8

% Part A
load('Data/SineResponseP6a');
TimeP6a = [((SineResponseP6a(124:124+5000,1) - SineResponseP6a(124,1)) /...
    1000)];
OutputP6a = [SineResponseP6a(124:124+5000,2)];

% From Exp 1
simOut6a = sim('PD_Control_Sine02.slx');
thetaL6a(1,:) = simOut6a.yout{1}.Values.Data';
tSpan6a(1,:) = simOut6a.yout{1}.Values.Time';
thetaL6a(2,:) = simOut6a.yout{2}.Values.Data';
tSpan6a(2,:) = simOut6a.yout{2}.Values.Time';
figure('Position', [200 200 1000 800]);
hold on;
grid on;
plot(tSpan6a(1,:),thetaL6a(1,:))
plot(TimeP6a, OutputP6a)
plot(tSpan6a(2,:),thetaL6a(2,:))
legend({'Simulated \theta_L [rad]','Experimental \theta_L [rad]',...
    'Sine Input [rad]'},'Location','northeast')
title('Response to 0.2 Hz Input Sine Wave')
xlabel('Time [s]')

% Part B
load('Data/SineResponseP6b');
TimeP6b = [((SineResponseP6b(210:210+5000,1) - SineResponseP6b(210,1)) /...
    1000)];
OutputP6b = [-SineResponseP6b(210:210+5000,2)];

% From Exp 1
simOut6b = sim('PD_Control_Sine2.slx');
thetaL6b(1,:) = simOut6b.yout{1}.Values.Data';
tSpan6b(1,:) = simOut6b.yout{1}.Values.Time';
thetaL6b(2,:) = simOut6b.yout{2}.Values.Data';
tSpan6b(2,:) = simOut6b.yout{2}.Values.Time';
figure('Position', [200 200 1000 800]);
hold on;
grid on;
plot(tSpan6b(1,:),thetaL6b(1,:))
plot(TimeP6b,OutputP6b)
plot(tSpan6b(2,:),thetaL6b(2,:))
legend({'Simulated \theta_L [rad]','Experimental \theta_L [rad]',...
    'Power Amp Input [V]'},'Location','northeast')
title('Response to 2 Hz Input Sine Wave')
xlabel('Time [s]')