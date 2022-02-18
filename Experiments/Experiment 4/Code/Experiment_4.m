%% Experiment 4
% William Watkins
% 5 February 2022

%% Clean up
clear all
close all
clc

%% Constants
m1 = 10;
m2 = 350;
Kw = 500000;
Ks = 10000;
b = 0;
Input = 1;
tspan = 0:0.001:5;
Initial = [0; 0; 0; 0];

%% Try in LTI SS model
A = [-(b/m1) (b/m1) -(1/m1) (1/m1);
    (b/m2) -(b/m2) 0 -(1/m2);
    Kw 0 0 0;
    -Ks Ks 0 0];

B = [0;0;-Kw;0];
C = [1 0 0 0;
     0 1 0 0];
D = [0;0];

%% Simulating
simOut = sim('Car_Suspension_Model.slx');
output(1,:) = simOut.yout{1}.Values.Data(:,2)';
tSpan(1,:) = simOut.yout{1}.Values.Time';
output(2,:) = simOut.yout{2}.Values.Data(:,2)';
tSpan(2,:) = simOut.yout{2}.Values.Time';
Final = simOut.yout{3}.Values.Data(:,2)';
tfinal = simOut.yout{3}.Values.Time';
output(4,:) = simOut.yout{4}.Values.Data';
tSpan(4,:) = simOut.yout{4}.Values.Time';

figure('Position', [200 200 1000 800]);
hold on;
grid on;
plot(tfinal,Final)

b = 2000;
A = [-(b/m1) (b/m1) -(1/m1) (1/m1);
    (b/m2) -(b/m2) 0 -(1/m2);
    Kw 0 0 0;
    -Ks Ks 0 0];
simOut = sim('Car_Suspension_Model.slx');
Final = simOut.yout{3}.Values.Data(:,2)';
tfinal = simOut.yout{3}.Values.Time';
plot(tfinal,Final)

b = 4000;
A = [-(b/m1) (b/m1) -(1/m1) (1/m1);
    (b/m2) -(b/m2) 0 -(1/m2);
    Kw 0 0 0;
    -Ks Ks 0 0];
simOut = sim('Car_Suspension_Model.slx');
Final = simOut.yout{3}.Values.Data(:,2)';
tfinal = simOut.yout{3}.Values.Time';
plot(tfinal,Final)

b = 4000;
A = [-(b/m1) (b/m1) -(1/m1) (1/m1);
    (b/m2) -(b/m2) 0 -(1/m2);
    Kw 0 0 0;
    -Ks Ks 0 0];
simOut = sim('Car_Suspension_Model.slx');
Final = simOut.yout{3}.Values.Data(:,2)';
tfinal = simOut.yout{3}.Values.Time';
plot(tfinal,Final)

b = 6000;
A = [-(b/m1) (b/m1) -(1/m1) (1/m1);
    (b/m2) -(b/m2) 0 -(1/m2);
    Kw 0 0 0;
    -Ks Ks 0 0];
simOut = sim('Car_Suspension_Model.slx');
Final = simOut.yout{3}.Values.Data(:,2)';
tfinal = simOut.yout{3}.Values.Time';
plot(tfinal,Final)

b = 8000;
A = [-(b/m1) (b/m1) -(1/m1) (1/m1);
    (b/m2) -(b/m2) 0 -(1/m2);
    Kw 0 0 0;
    -Ks Ks 0 0];
simOut = sim('Car_Suspension_Model.slx');
Final = simOut.yout{3}.Values.Data(:,2)';
tfinal = simOut.yout{3}.Values.Time';
plot(tfinal,Final)

b = 10000;
A = [-(b/m1) (b/m1) -(1/m1) (1/m1);
    (b/m2) -(b/m2) 0 -(1/m2);
    Kw 0 0 0;
    -Ks Ks 0 0];
simOut = sim('Car_Suspension_Model.slx');
Final = simOut.yout{3}.Values.Data(:,2)';
tfinal = simOut.yout{3}.Values.Time';
plot(tfinal,Final)

b = 12000;
A = [-(b/m1) (b/m1) -(1/m1) (1/m1);
    (b/m2) -(b/m2) 0 -(1/m2);
    Kw 0 0 0;
    -Ks Ks 0 0];
simOut = sim('Car_Suspension_Model.slx');
Final = simOut.yout{3}.Values.Data(:,2)';
tfinal = simOut.yout{3}.Values.Time';
plot(tfinal,Final)

legend({'b = 0','b=2,000','b=4,000','b=6,000','b=8,000','b=10,000'...
    ,'b=12,000'},'Location','northeast')
title('Car Body Acceleration in Response to Step Input')
xlabel('Time [s]')
ylabel('m/s^2')