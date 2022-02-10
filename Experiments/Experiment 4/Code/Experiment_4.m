%% Experiment 4
% William Watkins
% 5 February 2022

%% Clean up
clear all
close all
clc

%% Constants
m1 = 20;
m2 = 375;
Kw = 1000000;
Ks = 130000;
B = 9800;
Input = 1;
tspan = 0:0.01:10;
Initial = [0; 0; 0; 0];

%% Settings
options = odeset('AbsTol',1e-9,'RelTol',1e-9);

%% Running ODE45
[t, y] = ode45(@(t,y) Problem2_5(t,y,Input,m1,m2,Kw,Ks,B), tspan, Initial, options);
plot(t,y);

%% Try in LTI SS model
A = [-(B/m1) (B/m1) (1/m1) -(1/m1);
    (B/m2) -(B/m2) 0 -(1/m2);
    Kw 0 0 0;
    -Ks Ks 0 0];
