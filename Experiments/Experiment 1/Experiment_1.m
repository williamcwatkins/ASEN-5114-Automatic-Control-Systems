%% Experiment 1
% William Watkins
% 16 January 2022

%% Constants/Properties/Problem 1
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

%% Problem 2
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
s3 = (Jeq/(N*Ktau))*Lm;
s2 = (Jeq/(N*Ktau))*Rm;
s1 = N*Kb;

simOut1 = sim('Experiment_1_Model.slx');
thetaL1(1,:) = simOut1.yout{1}.Values.Data';
tSpan1(1,:) = simOut1.yout{1}.Values.Time';
thetaL1(2,:) = simOut1.yout{2}.Values.Data';
tSpan1(2,:) = simOut1.yout{2}.Values.Time';
figure('Position', [200 200 1000 800]);
hold on;
grid on;
yyaxis left
plot(tSpan1(1,:),thetaL1(1,:))
yyaxis right
plot(tSpan1(2,:),thetaL1(2,:))
legend({'\Theta_L [rad]','Power Amp Input [V]'},'Location','northeast')
title('Open Loop Response')
xlabel('Time [s]')

%% Question 4/5
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
yyaxis left
plot(tSpan5(1,:),thetaL5(1,:))
yyaxis right
plot(tSpan5(2,:),thetaL5(2,:))
legend({'\Theta_L [rad]','Power Amp Input [V]'},'Location','northeast')
title('Closed Loop Response')
xlabel('Time [s]')

%% Question 6
simOut6a = sim('PD_Control_Sine02.slx');
thetaL6a(1,:) = simOut6a.yout{1}.Values.Data';
tSpan6a(1,:) = simOut6a.yout{1}.Values.Time';
thetaL6a(2,:) = simOut6a.yout{2}.Values.Data';
tSpan6a(2,:) = simOut6a.yout{2}.Values.Time';
figure('Position', [200 200 1000 800]);
hold on;
grid on;
yyaxis left
plot(tSpan6a(1,:),thetaL6a(1,:))
yyaxis right
plot(tSpan6a(2,:),thetaL6a(2,:))
legend({'\Theta_L [rad]','Power Amp Input [V]'},'Location','northeast')
title('Closed Loop Response to 0.2 Hz')
xlabel('Time [s]')

simOut6b = sim('PD_Control_Sine2.slx');
thetaL6b(1,:) = simOut6b.yout{1}.Values.Data';
tSpan6b(1,:) = simOut6b.yout{1}.Values.Time';
thetaL6b(2,:) = simOut6b.yout{2}.Values.Data';
tSpan6b(2,:) = simOut6b.yout{2}.Values.Time';
figure('Position', [200 200 1000 800]);
hold on;
grid on;
yyaxis left
plot(tSpan6b(1,:),thetaL6b(1,:))
ylim([-0.4 0.4])
yyaxis right
plot(tSpan6b(2,:),thetaL6b(2,:))
ylim([-0.4 0.4])
legend({'\Theta_L [rad]','Power Amp Input [V]'},'Location','northeast')
title('Closed Loop Response to 2 Hz')
xlabel('Time [s]')