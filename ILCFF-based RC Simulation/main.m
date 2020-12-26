close all;
clc;
clear;

Color = [251  180  185;
         247  104  161;
         174  1    126;
         0    0    0
         ]/255;

%% Load plant model and Ctrller

load ILCFF_ctrl; 

Ts = 1/Hz ;   % unit: sec
P = GG;       % plant

%% Reference Setup 

RefFreq = 400;                  % desired frequency of the periodic signal
TriPulseAmp = 5;                % Tri wave
TriPulseT = 1/RefFreq;   
SinePulseAmp = 0;               % Sine wave

%% Run simulation 

sim('RepCtrlScanner',1); % º“¿¿1s


%% Tracking Result

figure
k = 0.00005:0.00005: 0.00005*(length(y));
plot(k,ref,'Linewidth',2,'Color',[167,209,41]/255);hold on;
plot(k,y0,'Linewidth',1,'Color',[0.9290    0.6940    0.1250]); hold on;
plot(k,y,'Linewidth',1,'Color',[0    0.4470    0.7410]); hold on; grid on;
xlabel('Time [s]','Interpreter','Latex');ylabel('Position [deg]','Interpreter','Latex'); 
axis([0.05, 0.06, -8, 8]);
legend('Reference','Open-Loop','RC with $F_{NLIIC}$','Interpreter','Latex')
title('RC Tracking Result (400Hz, $5^{o}$ Triangular Wave)','Interpreter','Latex');
set(gcf,'position',[500 500 595 320])

%% Tracking Error
figure
k = 0.00005:0.00005: 0.00005*(length(y));

plot(k,e0,'Linewidth',1,'Color',[0.9290    0.6940    0.1250]); hold on;
plot(k,e,'Linewidth',1,'Color',[0    0.4470    0.7410]); hold on; grid on;
xlabel('Time [s]','Interpreter','Latex');ylabel('Position [deg]','Interpreter','Latex'); 
axis([0.05, 0.06, -8, 8]);
legend('Open-Loop','RC with $F_{NLIIC}$','Interpreter','Latex')
title('RC Tracking Error (400Hz, $5^{o}$ Triangular Wave)','Interpreter','Latex');
set(gcf,'position',[500 500 595 320])
