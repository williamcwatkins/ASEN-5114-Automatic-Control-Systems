%% Experiment 5
% William Watkins
% 17 February 2022

%% Clean up
clear all
close all
clc

%% Problem 2
I = 0.1;
Gd = 0.9;
Gp = 1;

%% Problem 3
simOut = sim('Rigid_Body_Spacecraft.slx');
output(1,:) = simOut.yout{1}.Values.Data';
tSpan(1,:) = simOut.yout{1}.Values.Time';
output(3,:) = simOut.yout{2}.Values.Data';
output(2,:) = simOut.yout{3}.Values.Data';

figure('Position', [200 200 1000 800]);
title('Rigid Body Spacecraft with PD Control')
hold on;
grid on;
plot(tSpan, output(2,:));
plot(tSpan, output(3,:));
plot(tSpan, output(1,:));
legend({'Reference Torque [Nm]','Commanded Torque [Nm]','Spacecraft Angular Velocity [rad/s]'})