clc;
clear all;
close all;
warning('off','all');
rmpath('folderthatisnotonpath');

Color = [251  180  185;
         247  104  161;
         174  1    126;
         0    0    0
         ]/255;
 LineW = [2 2 2 2];

%% prepare plant model



Ts = 0.00005;
Hz = 1/Ts;

GG = zpk([-1550], [-2170 -2250 -6000],2170*2250*6000*0.98/1550);

GG = c2d(GG,Ts);

[numGG, denGG] = tfdata(GG, 'v');   






%%  ILCFF:  reference model design


% zero phase low pass filter design
SimBandwidth  = 700;
SimLength     = 94;  % FIR order of F(z) 
SimIter = 200; % iteration of ILC to learn reference impulse

nd = 0.5*SimLength;
N = 47;        % FIR filter order of N(z), M(z)=N(z)N(z^-1)
Fp  = SimBandwidth;   % passband-edge frequency
Fs  = Hz;         %  sampling frequency
Rp  = 0.00057565;  % Corresponds to 0.01 dB peak-to-peak ripple
Rst = 1e-4;        % Corresponds to 80 dB stopband attenuation
NUM = firceqrip(N,Fp/(Fs/2),[Rp Rst],'passedge'); % Fp/(Fs/2) is normalized frequency; NUM = vector of coeffs
Hd = dsp.FIRFilter('Numerator',NUM); 
%fvtool(Hd)

[numF_LP, denF_LP] = tf(Hd);


% FIR_LP_adj = tf(numF_LP,denF_LP,Ts)';
% FIR_LP_adj

F_LP = tf(numF_LP,denF_LP,Ts)*tf(numF_LP,denF_LP,Ts)'*tf([zeros(1, N),1],[1,zeros(1,N)],Ts); % F_LP, 2*N order causal filter 
[numF_LP, denF_LP] = tfdata(F_LP, 'v');
DelayF_LP = N;  % The max amp. of impulse is at N

% reference model M(z)
MD = F_LP/dcgain(F_LP) * tf([zeros(1, nd-N),1],[1,zeros(1,nd-N)],Ts); % Delay F_LP : z^-(nd-N) ,The max amp. of impulse is at N-N+nd=nd
td = SimLength*Ts;     % length of step response [sec]
MD_Imp = impulse(MD,td*2)*Ts;
% MD_Imp = MD_Imp/sum(MD_Imp);

% figure
% t = 1:length(MD_Imp);
% plot(t,MD_Imp);

%% ILC to track MD

% reference impulse
ILC_iter = SimIter;
ref_gain =1/max(MD_Imp);
r = ref_gain * MD_Imp(2:end)';

% figure
% t = 1:length(r);
% plot(t,r);



% Proposed ILC Setup
N = length(r);

R=fft(r);
n = 5; % update period
alpha = 0.15; % time-reveral learning gain

A = 0.01; % amplitude of output disturbances
noise= A*randn(1,N); 
D= fft(noise);
delta = max(abs(D));


u = 1*r;  %initial control input u
y = zeros(1, N);
e = zeros(1, N);
Y = zeros(ILC_iter, N);
U = zeros(ILC_iter, N);
E = zeros(ILC_iter, N);



RMSE =zeros(1,SimIter);
E_max = zeros(1,ILC_iter);
E_rms = E_max;
U_max = zeros(1,ILC_iter);
U_rms = U_max;



disp('ILC Learning in Progress...');



for ii = 1:SimIter
    %ii
    
    
    noise=A*randn(1,length(r));
    
    U(ii,:) = u;
    
    y = filter(numGG, denGG, u)+noise; % output signal

    e = r - y;
    ee=norm(e,2);
    RMS_error=sqrt((ee^2)/length(e));
    RMSE(1,ii)=RMS_error;  
    E_max(ii) = max(abs(e));
    
if (mod(ii,n))~=0  % time-reversal
    
    noise=A*randn(1,length(r));
    ee=filter(numGG, denGG, fliplr(e))+noise;
    Le = alpha*fliplr(ee);
    u=u+Le;

  
else  % proposed ILC
    
    
    YY=fft(y);
    UU = fft(u); %control input in frequncy domain
    EE = fft(e); %error signal in frequncy domain

    
    for k=1:length(R)            %proposed algorithm
     
     if  abs(R(k))< abs(EE(k))+delta
     
     UU(k)=UU(k); 
             
     else
         
     UU(k)=UU(k)+(UU(k)/R(k))*EE(k);  
    
     end   
    end
     
    
    u=ifft(UU);

   
end



end



Iter2plot = ILC_iter;




%% ILC Learning Result (track reference impulse)
figure;
subplot(2,1,1);
t = 0*0.00005:0.00005:(length(r)-1)*0.00005;
plot(t,r,'Linewidth',2.5,'Color',[167,209,41]/255);
hold on;
plot(t,y,'Linewidth',1,'Color',[0.9290    0.6940    0.1250]);
hold on;
xlabel('Time [s]','Interpreter','Latex');ylabel('Position [degree]','Interpreter','Latex');
axis([0, 0.0094, -0.25, 1.1]);
legend('reference impulse','learned output(NLIIC)','Interpreter','Latex')
title('Impulse Signal Tracking','Interpreter','Latex');grid on;
%set(gcf,'position',[500 500 455 220])
e = r-y;
subplot(2,1,2);
t = 0*0.00005:0.00005:(length(r)-1)*0.00005;
plot(t,e,'Linewidth',1,'Color',[0.9290    0.6940    0.1250]);
hold on;
xlabel('Time [s]','Interpreter','Latex');ylabel('Error [degree]','Interpreter','Latex');
axis([0, 0.0094, -0.1, 0.1]);
set(gcf,'position',[500 500 455 440]); grid on;

%% Learned Control Input    
figure;
t = 0:0.00005:0.00005*(length(u)-1);
plot(t,u,'Linewidth',1,'Color',[ 0    0.4470    0.7410]);
grid on; hold on;
xlabel('Time [s]','Interpreter','Latex');ylabel('Impulse Response [degree]','Interpreter','Latex');
axis([0*0.00005, 187*0.00005, -1.6, 2.5]);
title('Inverse Filter','Interpreter','Latex');
set(gcf,'position',[500 500 455 320])
     
     
%% ILC Learning Curve     
     
figure
t=1:length(RMSE);
loglog(t,RMSE,'Color',Color(1,:),'Linewidth',LineW(1)); hold on;
grid on;
title('ILC Learning Curve');
ylabel('RMS error [mm]');xlabel('iteration number');
axis([0, SimIter, 0, inf]);	
% legend('Time-reversal based','MFIIC','NLIIC','Proposed')
% set(legend,'location','best')
set(gcf,'position',[500 500 455 220])

%% Generate Inverse filter

ref_freq = 100;
N = Hz/(ref_freq);

numFF = [U(Iter2plot,1:0.5*end)] / (ref_gain);

% numFF = numFF / sum(numFF);
denFF = [1, zeros(1, size(numFF,2))];
sum(numFF)
FF = tf(numFF, denFF, Ts); % feedforward controller
FFdelay = nd;

%% Q-filter
q1 = [0.25 0.5 0.25];                               % z^-1 ... z^0 ... z^1
q2 = conv(q1,q1);                                   % z^-2 ... z^0 ... z^2
q4 = conv(q2,q2);                                   % z^-4 ... z^0 ... z^4
q8 = conv(q4,q4);                                   % z^-8 ... z^0 ... z^8
q16 = conv(q8,q8);                                  % z^-16... z^0 ... z^16
q32 = conv(q16,q16);                                % z^-32... z^0 ... z^32

% Q = tf(q2,[1 0 0 0 0],T);
Q = tf(q1,[1 0 0],Ts); % low-pass filter to 1000 Hz

figure 
bode(Q);

nq = 1;     
     


%% Repetitive Controller Design

delayK1 = FFdelay ;

delayN2 = delayK1; % total non-causal steps of F, N2=d+nu
delayN1 = N-delayN2-nq;


%% save controller
save ILCFF_ctrl Q FF nq delayN1 delayN2 N Hz GG;