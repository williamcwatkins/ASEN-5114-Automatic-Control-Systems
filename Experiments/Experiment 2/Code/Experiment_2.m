%% Experiment 2
% William Watkins
% 16 January 2022

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
Gd = Gd_o;
Gp = Gp_o;

%% Problem 1
pole1 = 0;
pole2 = (-(Jeq*Rm/(N*Ktau)) + sqrt((Jeq*Rm/(N*Ktau))^2 - 4*(Jeq*Lm/(N*Ktau))*(N*Kb)))/(2*Jeq*Lm/(N*Ktau));
pole3 = (-(Jeq*Rm/(N*Ktau)) - sqrt((Jeq*Rm/(N*Ktau))^2 - 4*(Jeq*Lm/(N*Ktau))*(N*Kb)))/(2*Jeq*Lm/(N*Ktau));

figure('Position', [200 200 1000 800]);
hold on;
grid on;
plot([pole1 pole2 pole3], [0 0 0], '*','color','red');
xlabel('Re(s)');
ylabel('Im(s)');
xlim([-11000 1000])

%% Problem 2
p = [Jeq*Lm/(N*Ktau) (Jeq*Rm/(N*Ktau)) N*Kb-Gd -Gp];
r = roots(p);
plot(r, [0 0 0], '*','color','magenta');

%% Problem 3
g = 0:0.01:1;
pg = [Jeq*Lm/(N*Ktau) (Jeq*Rm/(N*Ktau)) N*Kb-g(1)*Gd -g(1)*Gp];
rg(:,1) = roots(pg(1,:));
for i = 2:length(g)
    pg(i,:) = [Jeq*Lm/(N*Ktau) (Jeq*Rm/(N*Ktau)) N*Kb-g(i)*Gd -g(i)*Gp];
    rg(:,i) = roots(pg(i,:));
end
reax = zeros(3,101);
plot(rg,reax,'*','color','green')

%% Problem 4
g4 = 1:1:100;
pg4 = [Jeq*Lm/(N*Ktau) (Jeq*Rm/(N*Ktau)) N*Kb-g4(1)*Gd -g4(1)*Gp];
rg4(:,1) = roots(pg4(1,:));
for i = 2:length(g4)
    pg4(i,:) = [Jeq*Lm/(N*Ktau) (Jeq*Rm/(N*Ktau)) N*Kb-g4(i)*Gd -g4(i)*Gp];
    rg4(:,i) = roots(pg4(i,:));
end

plot(rg4,'*','color','blue')

%% Problem 5
g5 = 0:-0.01:-1;
pg5 = [Jeq*Lm/(N*Ktau) (Jeq*Rm/(N*Ktau)) N*Kb-g5(1)*Gd -g5(1)*Gp];
rg5(:,1) = roots(pg5(1,:));
for i = 2:length(g5)
    pg5(i,:) = [Jeq*Lm/(N*Ktau) (Jeq*Rm/(N*Ktau)) N*Kb-g5(i)*Gd -g5(i)*Gp];
    rg5(:,i) = roots(pg5(i,:));
end

plot(rg5,reax,'*','color','cyan')

%% Problem 6
g = [-1 -0.5 0 0.5 1 10 50 100];
Gd = g(1)*Gd_o;
Gp = g(1)*Gp_o;
simOut6 = sim('Models/PD_Control_Step.slx');
simTime1(1,:) = simOut6.yout{1}.Values.Time';
simVolt1(1,:) = simOut6.yout{1}.Values.Data';
simTime1(2,:) = simOut6.yout{2}.Values.Time';
simVolt1(2,:) = simOut6.yout{2}.Values.Data';

Gd = g(2)*Gd_o;
Gp = g(2)*Gp_o;
simOut6 = sim('Models/PD_Control_Step.slx');
simTime2(1,:) = simOut6.yout{1}.Values.Time';
simVolt2(1,:) = simOut6.yout{1}.Values.Data';
simTime2(2,:) = simOut6.yout{2}.Values.Time';
simVolt2(2,:) = simOut6.yout{2}.Values.Data';

Gd = g(3)*Gd_o;
Gp = g(3)*Gp_o;
simOut6 = sim('Models/PD_Control_Step.slx');
simTime3(1,:) = simOut6.yout{1}.Values.Time';
simVolt3(1,:) = simOut6.yout{1}.Values.Data';
simTime3(2,:) = simOut6.yout{2}.Values.Time';
simVolt3(2,:) = simOut6.yout{2}.Values.Data';

Gd = g(4)*Gd_o;
Gp = g(4)*Gp_o;
simOut6 = sim('Models/PD_Control_Step.slx');
simTime4(1,:) = simOut6.yout{1}.Values.Time';
simVolt4(1,:) = simOut6.yout{1}.Values.Data';
simTime4(2,:) = simOut6.yout{2}.Values.Time';
simVolt4(2,:) = simOut6.yout{2}.Values.Data';

Gd = g(5)*Gd_o;
Gp = g(5)*Gp_o;
simOut6 = sim('Models/PD_Control_Step.slx');
simTime5(1,:) = simOut6.yout{1}.Values.Time';
simVolt5(1,:) = simOut6.yout{1}.Values.Data';
simTime5(2,:) = simOut6.yout{2}.Values.Time';
simVolt5(2,:) = simOut6.yout{2}.Values.Data';

Gd = g(6)*Gd_o;
Gp = g(6)*Gp_o;
simOut6 = sim('Models/PD_Control_Step.slx');
simTime6(1,:) = simOut6.yout{1}.Values.Time';
simVolt6(1,:) = simOut6.yout{1}.Values.Data';
simTime6(2,:) = simOut6.yout{2}.Values.Time';
simVolt6(2,:) = simOut6.yout{2}.Values.Data';

Gd = g(7)*Gd_o;
Gp = g(7)*Gp_o;
simOut6 = sim('Models/PD_Control_Step.slx');
simTime7(1,:) = simOut6.yout{1}.Values.Time';
simVolt7(1,:) = simOut6.yout{1}.Values.Data';
simTime7(2,:) = simOut6.yout{2}.Values.Time';
simVolt7(2,:) = simOut6.yout{2}.Values.Data';

Gd = g(8)*Gd_o;
Gp = g(8)*Gp_o;
simOut6 = sim('Models/PD_Control_Step.slx');
simTime8(1,:) = simOut6.yout{1}.Values.Time';
simVolt8(1,:) = simOut6.yout{1}.Values.Data';
simTime8(2,:) = simOut6.yout{2}.Values.Time';
simVolt8(2,:) = simOut6.yout{2}.Values.Data';

%% PLots for 6
figure('Position', [200 200 1000 800]);
hold on;
grid on;
yyaxis left
plot(simTime1(1,:),simVolt1(1,:))
yyaxis right
plot(simTime1(2,:),simVolt1(2,:))
legend({'\theta_L [rad]','Power Amp Input [V]'},'Location','northeast')
title('Closed Loop Response, g = -1')
xlabel('Time [s]')

figure('Position', [200 200 1000 800]);
hold on;
grid on;
yyaxis left
plot(simTime2(1,:),simVolt2(1,:))
yyaxis right
plot(simTime2(2,:),simVolt2(2,:))
legend({'\theta_L [rad]','Power Amp Input [V]'},'Location','northeast')
title('Closed Loop Response, g = -0.5')
xlabel('Time [s]')

figure('Position', [200 200 1000 800]);
hold on;
grid on;
yyaxis left
plot(simTime3(1,:),simVolt3(1,:))
yyaxis right
plot(simTime3(2,:),simVolt3(2,:))
legend({'\theta_L [rad]','Power Amp Input [V]'},'Location','northeast')
title('Closed Loop Response, g = 0')
xlabel('Time [s]')

figure('Position', [200 200 1000 800]);
hold on;
grid on;
yyaxis left
plot(simTime4(1,:),simVolt4(1,:))
yyaxis right
plot(simTime4(2,:),simVolt4(2,:))
legend({'\theta_L [rad]','Power Amp Input [V]'},'Location','northeast')
title('Closed Loop Response, g = 0.5')
xlabel('Time [s]')

figure('Position', [200 200 1000 800]);
hold on;
grid on;
yyaxis left
plot(simTime5(1,:),simVolt5(1,:))
yyaxis right
plot(simTime5(2,:),simVolt5(2,:))
legend({'\theta_L [rad]','Power Amp Input [V]'},'Location','northeast')
title('Closed Loop Response, g = 1')
xlabel('Time [s]')

figure('Position', [200 200 1000 800]);
hold on;
grid on;
yyaxis left
plot(simTime6(1,:),simVolt6(1,:))
yyaxis right
plot(simTime6(2,:),simVolt6(2,:))
ylim([0 1.2])
legend({'\theta_L [rad]','Power Amp Input [V]'},'Location','northeast')
title('Closed Loop Response, g = 10')
xlabel('Time [s]')

figure('Position', [200 200 1000 800]);
hold on;
grid on;
yyaxis left
plot(simTime7(1,:),simVolt7(1,:))
yyaxis right
plot(simTime7(2,:),simVolt7(2,:))
ylim([0 1.2])
legend({'\theta_L [rad]','Power Amp Input [V]'},'Location','northeast')
title('Closed Loop Response, g = 50')
xlabel('Time [s]')

figure('Position', [200 200 1000 800]);
hold on;
grid on;
yyaxis left
plot(simTime8(1,:),simVolt8(1,:))
yyaxis right
plot(simTime8(2,:),simVolt8(2,:))
ylim([0 1.2])
legend({'\theta_L [rad]','Power Amp Input [V]'},'Location','northeast')
title('Closed Loop Response, g = 100')
xlabel('Time [s]')