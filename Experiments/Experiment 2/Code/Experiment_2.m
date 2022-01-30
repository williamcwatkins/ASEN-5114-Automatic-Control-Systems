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
Gd = -0.1;
Gp = -10;

%% Problem 1
pole1 = 0;
pole2 = (-(Jeq*Rm/(N*Ktau)) + sqrt((Jeq*Rm/(N*Ktau))^2 - 4*(Jeq*Lm/(N*Ktau))*(N*Kb)))/(2*Jeq*Lm/(N*Ktau));
pole3 = (-(Jeq*Rm/(N*Ktau)) - sqrt((Jeq*Rm/(N*Ktau))^2 - 4*(Jeq*Lm/(N*Ktau))*(N*Kb)))/(2*Jeq*Lm/(N*Ktau));

figure('Position', [200 200 1000 800]);
hold on;
grid on;
plot([pole1 pole2 pole3], [0 0 0], '*');
title('Poles');
xlabel('Re(s)');
ylabel('Im(s)');

%% Problem 2
p = [Jeq*Lm/(N*Ktau) (Jeq*Rm/(N*Ktau)) N*Kb-Gd -Gp];
r = roots(p);
plot(r, [0 0 0], '*');

%% Problem 3
g = 0:0.01:1;
pg = [Jeq*Lm/(N*Ktau) (Jeq*Rm/(N*Ktau)) N*Kb-g(1)*Gd -g(1)*Gp];
rg(:,1) = roots(pg(1,:));
for i = 2:length(g)
    pg(i,:) = [Jeq*Lm/(N*Ktau) (Jeq*Rm/(N*Ktau)) N*Kb-g(i)*Gd -g(i)*Gp];
    rg(:,i) = roots(pg(i,:));
end
reax = zeros(3,101);

% roots are all real, so can just plot on Real axis
plot(rg,reax,'*','MarkerFaceColor',[0 0.447 0.741])
% set(gca, 'XScale', 'log');
xlim([-11000 0])

%% Problem 4
g4 = 0:1:100;
pg4 = [Jeq*Lm/(N*Ktau) (Jeq*Rm/(N*Ktau)) N*Kb-g4(1)*Gd -g4(1)*Gp];
rg4(:,1) = roots(pg4(1,:));
for i = 2:length(g4)
    pg4(i,:) = [Jeq*Lm/(N*Ktau) (Jeq*Rm/(N*Ktau)) N*Kb-g4(i)*Gd -g4(i)*Gp];
    rg4(:,i) = roots(pg4(i,:));
end

plot(rg4,'*','MarkerFaceColor',[0 0.447 0.741])
% set(gca, 'XScale', 'log');
% set(gca, 'YScale', 'log');
