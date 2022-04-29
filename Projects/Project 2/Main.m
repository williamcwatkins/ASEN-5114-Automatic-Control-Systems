 %% Housekeeping
clc
clear
close all

%% Problem Setup
%n = 2.4
 n = 0.75;
m1 = n*0.09; % tip mass 1, [kg]
m2 = n*0.09; % tip mass 2, [kg]
Jb = n*0.01; % body MOI, [kg m^2]
k1 = n*.115; % wing 1 hinge spring, [Nm/rad]
k2 = n*.115; % wing 2 hinge spring, [Nm/rad]
l1 = 0.3; % wing 1 length, [m]
l2 = 0.3; % wing 2 length, [m]
r = 0.071; % body radius to wing attachment, [m]

A = [0 0 0 (1+r/l1)/Jb (1+r/l2)/Jb  0;
  0 0 0 -1/(l1*m1)  0        0;
  0 0 0  0  -1/(l2*m2)   0;
  -k1*(1+r/l1) k1/l1 0 0 0    0;
  -k2*(1+r/l2) 0 k2/l2 0 0    0;
       1            0      0    0   0    0];
B = [1/Jb; 0; 0; 0; 0; 0];
C = [0  0  0  0 0  1];
D = [0];
 
sc_plant = ss(A,B,C,D);

%% Parse Experimental Data
[Data,FileName,DataNumInfo] = parseData();

%% Convert, Organize Data
% Let's get the data into the form we want.
% First, let's find the angular position of the body from the gyro reading
ExpCombFreqHz = [];
ExpCombAngMagDB = [];
ExpCombAngVelMagDB = [];
Stackmag = [];
TimeComb = [];
MagComb = [];
PhaseExp = [];

for i = 1:length(Data) % For every data set
    
    Data{i}(:,4) = 2*pi*Data{i}(:,1); % Frequency in rad/s
    Data{i}(:,2) = Data{i}(:,2)*1000./(2*pi*(Data{i}(:,1))); % Converting from mNm to Nm
    
    Data{i}(:,5) = 1./Data{i}(:,1); % Finding the time values from the frequency
    
    PhaseExp = [PhaseExp;Data{i}(:,3)*180/pi-90];
    
        
    freqRangeHz = Data{i}(:,1); % [Hz]
    MagComb = [MagComb;Data{i}(:,2)];
    ExpCombFreqHz = [ExpCombFreqHz;freqRangeHz]; % [Hz]
    ExpCombAngVelMagDB = [ExpCombAngVelMagDB;20*log10(Data{i}(:,2))]; % [dB]
    
end


[ExpCombFreqHz,CombFreqOrder] = sort(ExpCombFreqHz);

ExpCombAngVelMagDB = ExpCombAngVelMagDB(CombFreqOrder,:);
PhaseExp = PhaseExp(CombFreqOrder,:);

    
%% Problem 1: Matching Analytical to Experimental - add damping
% Check controllability and observability of plant system
nsysNonMin = rank(ctrb(A,B));
osysNonMin = rank(obsv(A,C));

% Find minimal state space system
NonMinSys = ss(A,B,C,D);
MinSys = minreal(NonMinSys);
APlantMin = MinSys.A;
BPlantMin = MinSys.B;
CPlantMin = MinSys.C;
DPlantMin = MinSys.D;

nsysMin = rank(ctrb(APlantMin,BPlantMin));
osysMin = rank(obsv(APlantMin,CPlantMin));

% Some constants to move poles
Re1 = -0.25;
Im1 = 7.5766i;

Re0 = -0.01;

% Some constants to move zeros
ZRe1 = -0.25;
ZIm1 = 4.4655i;

MinPlantPoles = eig(APlantMin);
[MinPlantsysnum,MinPlantsysden] = ss2tf(APlantMin,BPlantMin,CPlantMin,DPlantMin);
tfold = tf(MinPlantsysnum,MinPlantsysden);
[zplant,pplant,kplant] = tf2zp(MinPlantsysnum,MinPlantsysden);
znew = [ZRe1 + ZIm1, ZRe1 - ZIm1]'; % New Zeros
pnew = [0, Re0, Re1+Im1, Re1-Im1]; % New Poles
[NumNew,DenNew] = zp2tf(znew,pnew,kplant);
tfNew = tf(NumNew,DenNew);
[AMin,BMin,CMin,DMin] = tf2ss(NumNew,DenNew);


% Plotting Pole Locations
figure
subplot(1,2,1)
hold on
sz = 150;
scatter(real(pplant),imag(pplant),sz,'x','MarkerEdgeColor',[0 0 1],"LineWidth",3)
scatter(real(zplant),imag(zplant),sz,'o','MarkerEdgeColor',[1 0 0],"LineWidth",3)
ylabel("Im(s)")
xlabel("Re(s)")
xline(0,"LineWidth",2)
yline(0,"LineWidth",2)
legend("Plant Poles","Plant Zeros",'','','Location','northwest')
set(gca,"Fontsize",25)
xlim([-1,0.1])
ylim([-8,8])
title("Undamped Poles and Zeros in Complex S Plane")

subplot(1,2,2)
hold on
sz = 150;
scatter(real(pnew),imag(pnew),sz,'x','MarkerEdgeColor',[0 0 1],"LineWidth",3)
scatter(real(znew),imag(znew),sz,'o','MarkerEdgeColor',[1 0 0],"LineWidth",3)
ylabel("Im(s)")
xlabel("Re(s)")
xline(0,"LineWidth",2)
yline(0,"LineWidth",2)
legend("New Poles", "New Zeros",'','','Location','northwest')
set(gca,"Fontsize",25)
xlim([-1,0.1])
ylim([-8,8])
title("Damped Poles and Zeros in Complex S Plane")

% Plotting analytical data with experimental data
 w = logspace(-.2,2.0,1000);
 [mag,phase,w] = bode(tfNew,w);
 mag = squeeze(mag);
 phase = squeeze(phase);
 
  [magold,phaseold,wold] = bode(tfold,w);
 magold = squeeze(magold);
 phaseold = squeeze(phaseold);
 
  ind1 = find(w/(2*pi) < 0.6);
  phaseold(ind1) = phaseold(ind1)-360;
 
 figure;
 subplot(211)
 semilogx(ExpCombFreqHz,ExpCombAngVelMagDB,'x',"Linewidth",3)
 hold on
 semilogx(w/(2*pi),20*log10(mag),"Linewidth",3)
 ylabel('Magnitude, [rad/(Nm), dB]')
 title('Figure 2: analytic frequency response with estimated parameters')
 legend("Experimental","Analytical")
 grid
 
 subplot(212)
 semilogx(ExpCombFreqHz,PhaseExp,'x',"Linewidth",3)
 hold on
 semilogx(w/(2*pi),phase,"Linewidth",3)
 ylabel('Phase, [deg]')
 xlabel('Frequency, [Hz]')
  legend("Experimental","Analytical")
 grid
 
figure
subplot(2,2,1)
 semilogx(ExpCombFreqHz,ExpCombAngVelMagDB,'x',"Linewidth",3)
 hold on
 semilogx(w/(2*pi),20*log10(magold),"Linewidth",3)
 ylabel('Magnitude, [rad/(Nm), dB]')
 title('Frequency Response without Damping')
  legend("Experimental","Analytical")
 grid
 set(gca,"Fontsize",25)

subplot(2,2,2)
 semilogx(ExpCombFreqHz,ExpCombAngVelMagDB,'x',"Linewidth",3)
 hold on
 semilogx(w/(2*pi),20*log10(mag),"Linewidth",3)
 ylabel('Magnitude, [rad/(Nm), dB]')
 title('Frequency Response with Damping')
  legend("Experimental","Analytical")
 grid
 set(gca,"Fontsize",25)

subplot(2,2,3)
 semilogx(ExpCombFreqHz,PhaseExp,'x',"Linewidth",3)
 hold on
 semilogx(w/(2*pi),phaseold,"Linewidth",3)
 ylabel('Phase, [deg]')
 xlabel('Frequency, [Hz]')
  legend("Experimental","Analytical")
 grid
 title("Phase without damping")
  set(gca,"Fontsize",25)
 
subplot(2,2,4)
 semilogx(ExpCombFreqHz,PhaseExp,'x',"Linewidth",3)
 hold on
 semilogx(w/(2*pi),phase,"Linewidth",3)
 ylabel('Phase, [deg]')
 xlabel('Frequency, [Hz]')
  legend("Experimental","Analytical")
 grid
 title("Phase with damping")
 set(gca,"Fontsize",25)


%% Problem 2: Controller Design

poles = [-9.8+0.5i, -9.8-0.5i, -0.25+4.4655i, -0.25-4.4655i];
%poles = [-0.1, -0.2, -0.25+4.4655i, -0.25-4.4655i];
%poles = [-0.01+0.5i, -0.01-0.5i, -5+6i, -5-6i]
%poles = [-3, -4, -0.25+4.4655i, -0.25-4.4655i];

% Plot old poles and new poles
figure
subplot(1,2,1)
hold on
sz = 150;
scatter(real(pnew),imag(pnew),sz,'x','MarkerEdgeColor',[0 0 1],"LineWidth",3)
scatter(real(znew),imag(znew),sz,'o','MarkerEdgeColor',[1 0 0],"LineWidth",3)
ylabel("Im(s)")
xlabel("Re(s)")
xline(0,"LineWidth",2)
yline(0,"LineWidth",2)
legend("Plant Poles with Damping","Plant Zeros with Damping",'','','Location','northwest')
set(gca,"Fontsize",25)
xlim([-10,0.1])
ylim([-8,8])
title("Damped Plant Poles and Zeros in Complex S Plane")

subplot(1,2,2)
hold on
sz = 150;
scatter(real(poles),imag(poles),sz,'x','MarkerEdgeColor',[0 0 1],"LineWidth",3)
scatter(real(znew),imag(znew),sz,'o','MarkerEdgeColor',[1 0 0],"LineWidth",3)
ylabel("Im(s)")
xlabel("Re(s)")
xline(0,"LineWidth",2)
yline(0,"LineWidth",2)
legend("CL Poles", "CL Zeros",'','','Location','northwest')
set(gca,"Fontsize",25)
xlim([-10,0.1])
ylim([-8,8])
title("CL Poles and Zeros in Complex S Plane")


K = place(AMin,BMin,poles);
F = inv(CMin*inv(-AMin+BMin*K)*BMin);
I = eye(4,4);

ACL = AMin-BMin*K;
BCL = BMin*F;
CCL = CMin-DMin*K;
DCL = DMin*F;

[tfCLnum,tfCLden] = ss2tf(ACL,BCL,CCL,DCL);
tfCL = tf(tfCLnum,tfCLden);
T_s = tfCL;
%negLgtf = T_s/(1-T_s);

% Plotting Closed Loop Tracking Frequency Response
 w = logspace(-.2,2,1000);
 [mag,phase,w] = bode(tfCL,w);
 mag = squeeze(mag);
 phase = squeeze(phase);
 
  [magold,phaseold,wold] = bode(tfold,w);
 magold = squeeze(magold);
 phaseold = squeeze(phaseold);
 
  ind1 = find(w/(2*pi) < 0.6);
  phaseold(ind1) = phaseold(ind1)-360;

 magvec = 20*log10(mag);
 freqvec = w/(2*pi);
 vecdiff = abs(-3-magvec);
 [minval,indmin] = min(vecdiff);
 freq_3dB = freqvec(indmin)

 % Closed loop tracking frequency response (rad/rad)
 figure;
 semilogx(freqvec,magvec,"Linewidth",4)
 hold on
 yline(-3,'--k',"Linewidth",2)
 xline(freq_3dB,'--k',"Linewidth",2)
 scatter(1,-3,'m*',"Linewidth",5)
 ylabel('Magnitude, [dB rad/rad]')
 xlabel('Frequency [Hz]')
 legend("Frequency Response","-3 dB line and Cutoff Freq","","Min -3dB Cutoff",'location','southwest')
 title(['Closed Loop Controller Frequency Response with CL tracking BW ',num2str(freq_3dB),' Hz'])
 set(gca,"Fontsize",25)
 grid


% Plotting Closed loop frequency response from reference input to plant input
syms s

negLgA = AMin;
negLgB = BMin;
negLgC = K;
negLgD = 0;

negLgss = ss(negLgA,negLgB,negLgC,negLgD);



negLg = K*inv(s*eye(4)-AMin)*BMin;
%negLg = (K*adjoint(s*eye(4)-AMin)*BMin)/det(s*eye(4)-AMin);
%[negLgN, negLgD] = numden(negLg);
negLgN = K*adjoint(s*eye(4)-AMin)*BMin;
negLgD = det(s*eye(4)-AMin);
[negLgNCoeff,t] = coeffs(negLgN, s,'All');
[negLgDCoeff,t] = coeffs(negLgD, s,'All');
negLgNCoeff = double(negLgNCoeff);
negLgDCoeff = double(negLgDCoeff);
  negLgtf = tf(negLgNCoeff, negLgDCoeff);

  figure
  nyquist(negLgtf)

  figure
  h = nyquistplot(negLgtf);
  zoomcp(h)


u_r = (F)/(1+K*inv(s*eye(4)-AMin)*BMin);
%u_r = F-K*inv(s*eye(4)-AMin+BMin*K)*BMin*F;

 [u_rN, u_rD] = numden(u_r);

 [u_rNCoeff,t] = coeffs(u_rN, s,'All');
 [u_rDCoeff,t] = coeffs(u_rD, s,'All');
u_rNCoeff = double(u_rNCoeff);
u_rDCoeff = double(u_rDCoeff);

 % Transfer function from reference input to plant input (Nm/rad)
 u_rtf = tf(u_rNCoeff, u_rDCoeff);


 w = logspace(-1,2.5,10000);
 [mag,phase,w] = bode(u_rtf,w);
 mag = squeeze(mag);
 phase = squeeze(phase);
 
  [magold,phaseold,wold] = bode(tfold,w);
 magold = squeeze(magold);
 phaseold = squeeze(phaseold);
 
  ind1 = find(w/(2*pi) < 0.6);
  phaseold(ind1) = phaseold(ind1)-360;

 magvec = 20*log10(mag);
 freqvec = w/(2*pi);

 % Closed loop frequency response from reference input to plant input (Nm/rad)
 figure;
 semilogx(freqvec,magvec,"Linewidth",4)
 hold on
 ylabel('Magnitude, [dB Nm/rad]')
 xlabel('Frequency [Hz]')
 title('Closed loop frequency response from reference input to plant input')
 set(gca,"Fontsize",25)
 grid


% Plotting -Lg bode
 %w = linspace(-10000,10500,1000);
  %w = linspace(-0.2,2,1000);
 [mag,phase,w] = bode(negLgtf,w);
 mag = squeeze(mag);
 phase = squeeze(phase);

 magvec = 20*log10(mag);
 freqvec = w/(2*pi);

 figure;
 semilogx(freqvec,magvec,"Linewidth",4)
 hold on
 ylabel('-Lg, [dB]')
 xlabel('Frequency [Hz]')
 title("Closed Loop Negative Loop Gain Bode Plot")
 set(gca,"Fontsize",25)
 grid

% Plotting -Lg Nyquist
%w = logspace(-.2,10^100,1000);
 figure
 hold on
  [Re,Im,~] = nyquist(negLgtf,w);
  Re = squeeze(Re);
  Im = squeeze(Im);
nyquist(negLgtf,w)
 plot(Re,Im, "LineWidth",4)
 plot(Re,-Im, "LineWidth",4)
 scatter(-1,0,"LineWidth",4)
  title("Closed Loop Negative Loop Gain Nyquist Plot")
 set(gca,"Fontsize",25)
 %grid





%% Problem 3: Simulating Design

% Sim_Time = linspace(0,100,1000);
% r_step = [zeros(1,18),0.1*ones(1,982)];
% r_sine_p1 = 0.15*sin(2*pi*0.1*Sim_Time);
% r_sine_1 = 0.15*sin(2*pi*1*Sim_Time);
% 
% tau_threshold = 2*(33.5)/1000; % Convert current threshold to torque threshold
% 
% open_system('CLFeedback');
% [stepOut] = sim('CLFeedback',Sim_Time,[],[Sim_Time',r_step']);
% [sineP1Out] = sim('CLFeedback',Sim_Time,[],[Sim_Time',r_sine_p1']);
% [sine1Out] = sim('CLFeedback',Sim_Time,[],[Sim_Time',r_sine_1']);
% % 
% Sim_Out_Step_p1 = stepOut.yout{2}.Values.Data;
% Control_Input_Step = stepOut.yout{1}.Values.Data;
% 
% Sim_Out_Sine_p1Hz = sineP1Out.yout{2}.Values.Data;
% Control_Input_Sine_p1Hz = sineP1Out.yout{1}.Values.Data;
% 
% Sim_Out_Sine_1Hz = sine1Out.yout{2}.Values.Data;
% Control_Input_Sine_1Hz = sine1Out.yout{1}.Values.Data;
% 
% % Plots
% figure;
% subplot(3,1,1)
% hold on
% plot(Sim_Time,Sim_Out_Step_p1,"LineWidth",3)
% plot(Sim_Time,r_step,'--',"LineWidth",3)
% legend("Output y","Input r")
%  ylabel('Position [rad]')
%  xlabel('Time [sec]')
%  grid
%  title("0.1 rad step response")
%  set(gca,"Fontsize",25)
%  
%  subplot(3,1,2)
% hold on
% plot(Sim_Time,Sim_Out_Sine_p1Hz,"LineWidth",3)
% plot(Sim_Time,r_sine_p1,'--',"LineWidth",3)
% legend("Output y","Input r")
%  ylabel('Output Position [rad]')
%  xlabel('Time [sec]')
%  grid
%  title("0.1 Hz Sine wave response")
%  set(gca,"Fontsize",25)
%  
% subplot(3,1,3)
% hold on
% plot(Sim_Time,Sim_Out_Sine_1Hz,"LineWidth",3)
% plot(Sim_Time,r_sine_1,'--',"LineWidth",3)
% legend("Output y","Input r")
%  ylabel('Output Position [rad]')
%  xlabel('Time [sec]')
%  grid
%  title("Simulated Response: 1 Hz Sine wave")
%  set(gca,"Fontsize",25)
%  
% % Plot torque input
% figure;
% subplot(3,1,1)
% hold on
% plot(Sim_Time,Control_Input_Step,"LineWidth",3)
% plot(Sim_Time,tau_threshold*ones(size(Sim_Time)),'--r','LineWidth',3)
% plot(Sim_Time,-tau_threshold*ones(size(Sim_Time)),'--r','LineWidth',3)
% ylabel('Control Torque [Nm]')
% xlabel('Time [sec]')
% grid
% title('Torque Input Signal: Step Input')
% legend('Torque signal','Torque limit','')
% set(gca,"Fontsize",25)
% 
% subplot(3,1,2)
% hold on
% plot(Sim_Time,Control_Input_Sine_p1Hz,"LineWidth",3)
% plot(Sim_Time,tau_threshold*ones(size(Sim_Time)),'--r','LineWidth',3)
% plot(Sim_Time,-tau_threshold*ones(size(Sim_Time)),'--r','LineWidth',3)
% ylabel('Control Torque [Nm]')
% xlabel('Time [sec]')
% grid
% title('Torque Input Signal: 0.1 Hz Sine wave')
% legend('Torque signal','Torque limit','')
% set(gca,"Fontsize",25)
% 
% subplot(3,1,3)
% hold on
% plot(Sim_Time,Control_Input_Sine_1Hz,"LineWidth",3)
% plot(Sim_Time,tau_threshold*ones(size(Sim_Time)),'--r','LineWidth',3)
% plot(Sim_Time,-tau_threshold*ones(size(Sim_Time)),'--r','LineWidth',3)
% ylabel('Control Torque [Nm]')
% xlabel('Time [sec]')
% grid
% title('Torque Input Signal: 1 Hz Sine wave')
% legend('Torque signal','Torque limit','')
% set(gca,"Fontsize",25)

 %% Observer design

 % Desired settling time: t_s ~= 3/sigma
 t_s_d = 0.17*[3 2.9 2.8 2.7]; %Desired settling time
 p_d = -3./t_s_d; %Find desired pole locations

 L = place(AMin', CMin', p_d);
 L = L';

 syms s
 T1 = K*inv(s*eye(4)-AMin+L*CMin)*BMin;
 T2 = K*inv(s*eye(4)-AMin+L*CMin)*L;
 [N1, D] = numden(T1);
 [N2, ~] = numden(T2);
 [N1, t] = coeffs(N1, s, 'All');
 N1 = double(N1);
 [N2, t] = coeffs(N2, s, 'All');
 N2 = double(N2);
 [D, t] = coeffs(D, s, 'All');
 D = double(D);

 % Find plant transfer function
 Dmin = 0;
 [b, a] = ss2tf(AMin, BMin, CMin, Dmin);
 Np = poly2sym(b, s);
 Dp = poly2sym(a, s);
 P = Np/Dp;

 % Find full transfer function for -Lg1
 Lg1Func = (1/(1+T1))*T2*P;

 [Lg1N, Lg1D] = numden(Lg1Func);

 [Lg1NCoeff,t] = coeffs(Lg1N, s,'All');
 [Lg1DCoeff,t] = coeffs(Lg1D, s,'All');
 Lg1NCoeff = double(Lg1NCoeff);
 Lg1DCoeff = double(Lg1DCoeff);


 % Find Lg1 frequency response
 sysLg1 = tf(Lg1NCoeff, Lg1DCoeff);
    
 % Plot frequency response
 w = logspace(-.2,2.0,1000);
 [mag,phase,w] = bode(sysLg1 ,w);
 mag = squeeze(mag);
 phase = squeeze(phase);

 figure;
 subplot(211)
 semilogx(w/(2*pi),20*log10(mag),"Linewidth",3)
 ylabel('Magnitude, [rad/(Nm), dB]')
 title('System frequency response with observer in the loop')
 grid
 
 subplot(212)
 semilogx(w/(2*pi),phase,"Linewidth",3)
 ylabel('Phase, [deg]')
 xlabel('Frequency, [Hz]')
 grid

 figure
 nyquist(sysLg1);
 title('System Nyquist plot with observer in the loop')

  [Gm, Pm] = margin(sysLg1);
  Gm = 20*log10(Gm);

  %h=nyquistplot(sysLg1), then zoomcp(h)


  %% Problem 5: Simulating control with observer
  % Why is lg2 different with observer?