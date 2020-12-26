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

 %% Simulation settings

SimIter       = 1000 ;


 
%% prepare plant model
Ts = 0.001;

P = zpk([0.9],[1 1],0.01,Ts);
C = tf([20 -19.9],[1 0],Ts);



GG = feedback(P*C,1);
[numGG, denGG] = tfdata(GG, 'v');   

%% reference  signal

N=2000;
fs = 1/Ts;			% sample rate 
time = (0:N-1)/fs;		% time resolution in time-domain
r = 100*chirp(time,0,2,100)-100;		

r=[zeros(1,0.25*N) r zeros(1,0.25*N)]; % reference signal 


%% Noise Setup

A = 1;  % amplitude of noise 
noise=A*randn(1,length(r));
D=fft(noise);
Delta=max(1*abs(D));


%% ILC to learn the reference signal: Proposed ILC

E_rms_Proposed=zeros(1,SimIter);
E_max_Proposed=zeros(1,SimIter);


R = fft(r);
n = 5;
alpha = 0.08;
u = 1*r;  %initial control input u




disp('ILC Learning in Progress...');



for ii = 1:SimIter
    %ii
    
    
    noise=A*randn(1,length(r));
    
    y = filter(numGG, denGG, u)+noise; % output signal

    Y=fft(y);
    
    
    e = r - y;
    ee=norm(e,2);
    RMS_error=sqrt((ee^2)/length(e));
    E_rms_Proposed(1,ii)=RMS_error;  
    E_max_Proposed(ii) = max(abs(e));
    
if (mod(ii,n))~=0  
   
    noise=A*randn(1,length(r));
    ee=filter(numGG, denGG, fliplr(e))+noise;
    Le = alpha*fliplr(ee);
    u=u+Le;

  
else
    
    
    
    U=fft(u); %control input in frequncy domain
    E=fft(e); %error signal in frequncy domain

    
    for k=1:length(R)            %proposed algorithm
     
     if  abs(R(k))<abs(E(k))+Delta
     
     U(k)=U(k); 
             
     else
         
     U(k)=U(k)+(U(k)/R(k))*E(k);  
    
     end   
    end
     
    
    u=ifft(U);

   
end



end

disp('ILC Learning Complete')
  

e_Proposed = r-y;    
ee=norm(e_Proposed,2);
e_RMS_Proposed=sqrt((ee^2)/length(e));


%% ILC to learn the reference signal: MFIIC

E_rms_MFIIC=zeros(1,SimIter);
E_max_MFIIC= zeros(1,SimIter);


R=fft(r);
u = r; % initial control input






disp('ILC Learning in Progress...');

% figure
for ii = 1:SimIter   
  
    %ii
    
    
    noise=A*randn(1,length(r));
    
 
     
    y = filter(numGG, denGG, u)+noise;  

    
    
    e = r - y;
   
    ee=norm(e,2);
    RMS_error=sqrt((ee^2)/length(e));
    E_rms_MFIIC(1,ii)=RMS_error;  
    E_max_MFIIC(ii) = max(abs(e));


     
   

    
    U=fft(u);
    Y=fft(y);
    E=fft(e);
 
    
  for k=1:length(R)
     if abs(Y(k))==0
     
       
   
     else
     U(k)=(U(k)/Y(k))*R(k);    
     end   
  end


   

   u=ifft(U);


    



end

disp('ILC Learning Complete')
  

e_MFIIC = r-y;
ee=norm(e_MFIIC,2);
e_RMS_MFIIC=sqrt((ee^2)/length(e));

%% ILC to learn the reference signal: time-reversal

E_rms_timerever=zeros(1,SimIter);
E_max_timerever=zeros(1,SimIter);

u = r;  %initial control input u

disp('ILC Learning in Progress...');


for ii = 1:SimIter
    %ii
    
    
    noise=A*randn(1,length(r)) ;
    
    y = filter(numGG, denGG, u)+noise; % output signal


    
    
    e = r - y;
    ee=norm(e,2);
    RMS_error=sqrt((ee^2)/length(e));
    E_rms_timerever(1,ii)=RMS_error;  
    E_max_timerever(ii) = max(abs(e));
    
    noise=A*randn(1,length(r)) ;
    ee = filter(numGG, denGG, fliplr(e))+noise;
    Le = alpha*fliplr(ee);
    u=u+Le;

  
   
end

e_timerev = r-y;
ee=norm(e_timerev,2);
e_RMS_timerever=sqrt((ee^2)/length(e));

disp('ILC Learning Complete')

%

E_rms_timerever2=zeros(1,SimIter);
E_max_timerever=zeros(1,SimIter);

u = r;  %initial control input u

disp('ILC Learning in Progress...');


for ii = 1:SimIter
    %ii
    
    
    noise=A*randn(1,length(r)) ;
    
    y = filter(numGG, denGG, u)+noise; % output signal


    
    
    e = r - y;
    ee=norm(e,2);
    RMS_error=sqrt((ee^2)/length(e));
    E_rms_timerever2(1,ii)=RMS_error;  
    E_max_timerever(ii) = max(abs(e));
    
    noise=A*randn(1,length(r)) ;
    ee = filter(numGG, denGG, fliplr(e))+noise;
    Le = 1.2*fliplr(ee);
    u=u+Le;

  
   
end



disp('ILC Learning Complete')

%
E_rms_timerever3=zeros(1,SimIter);
E_max_timerever=zeros(1,SimIter);

u = r;  %initial control input u

disp('ILC Learning in Progress...');


for ii = 1:SimIter
    %ii
    
    
    noise=A*randn(1,length(r)) ;
    
    y = filter(numGG, denGG, u)+noise; % output signal


    
    
    e = r - y;
    ee=norm(e,2);
    RMS_error=sqrt((ee^2)/length(e));
    E_rms_timerever3(1,ii)=RMS_error;  
    E_max_timerever(ii) = max(abs(e));
    
    noise=A*randn(1,length(r)) ;
    ee = filter(numGG, denGG, fliplr(e))+noise;
    Le = 1.568*fliplr(ee);
    u=u+Le;

  
   
end



disp('ILC Learning Complete')


%% NLIIC
E_rms_NLIIC=zeros(1,SimIter);
E_max_NLIIC = zeros(1,SimIter);

R=fft(r);
u = 1*r;  %initial control input u


disp('ILC Learning in Progress...');


for ii = 1:SimIter
    ii
    
    
   
    noise=A*randn(1,length(r)); % measurement noise
    y = filter(numGG, denGG, u)+noise; % output signal

    
 
    
    e = r - y;
    ee=norm(e,2);
    RMS_error=sqrt((ee^2)/length(e));
    E_rms_NLIIC(1,ii)=RMS_error;  
    E_max_NLIIC(ii) = max(abs(e));

    Y=fft(y);
    U=fft(u); %control input in frequncy domain
    E=fft(e); %error signal in frequncy domain
    D=fft(noise);
    
    for k=1:length(R)           
     if  abs(Y(k))==0
        U(k)=U(k);
     
     else
         
         if abs(Y(k))>abs(R(k)) 
             gain=1;
         else
             gain=0.5*(1-cos(pi*(abs(Y(k))/abs(R(k)))));
         end
         U(k)=U(k)+gain*(U(k)/Y(k))*E(k);
 
     end   
    end
     
    
    u=ifft(U);

   
end

e_NLIIC = r-y;
ee=norm(e_NLIIC,2);
e_RMS_NLIIC=sqrt((ee^2)/length(e));

disp('ILC Learning Complete')


%% Simulation Result

 % reference signal
 figure;
 t = 0:Ts:(length(r)-1)*Ts;
 cyan=[0 0.5 1];
 plot(t,r, 'Color',cyan,'Linewidth',1);hold on;
 xlabel('Time [s]');ylabel('Position [mm]');
 title('100 Hz Chirp Reference');
 grid on;
 set(gcf,'position',[500 500 455 180]);
 
 
% tracking error in steady state 
figure;
t = 0:Ts:(length(r)-1)*Ts;
%plot(t,e_timerev , 'Color',[0.95 0.81 0],'Linewidth',1);hold on;
plot(t,e_MFIIC , 'Color',[254  160  165]/255,'Linewidth',1);hold on;
plot(t,e_NLIIC , 'Color',[0.95 0.81 0],'Linewidth',1);hold on;
plot(t,e_Proposed , 'Color',[0 0.71 1],'Linewidth',1);hold on;
xlabel('Time [s]');ylabel('Error [mm]');
legend('MFIIC','NLIIC','Proposed')
title('Tracking Error in the Steady State (with large disturbance)');
grid on;
set(gcf,'position',[500 500 455 220]);

%ILC RMS Error Convergence

figure
t=1:length(E_rms_Proposed);
loglog(t,E_rms_timerever,'Color',Color(1,:),'Linewidth',LineW(1)); hold on;
loglog(t,E_rms_MFIIC,'Color',Color(2,:),'Linewidth',LineW(2)); hold on;
loglog(t,E_rms_NLIIC,'Color',Color(3,:),'Linewidth',LineW(3)); hold on;
loglog(t,E_rms_Proposed,'Color',Color(4,:),'Linewidth',LineW(4)); hold on;
grid on;
title('Convergence of the Tracking Error (with large disturbance)');
ylabel('RMS error [mm]');xlabel('iteration number');
axis([0, SimIter, 0, inf]);	
legend('Time-reversal based','MFIIC','NLIIC','Proposed')
% set(legend,'location','best')
set(gcf,'position',[500 500 455 220])



figure
t=1:length(E_rms_Proposed);
loglog(t,E_rms_timerever,'Color',Color(1,:),'Linewidth',LineW(1)); hold on;
loglog(t,E_rms_MFIIC,'Color',Color(2,:),'Linewidth',LineW(2)); hold on;
loglog(t,E_rms_NLIIC,'Color',Color(3,:),'Linewidth',LineW(3)); hold on;
loglog(t,E_rms_Proposed,'Color',Color(4,:),'Linewidth',LineW(4)); hold on;
grid on;
axis([0, SimIter, 0, inf]);		


% time-reversal based ILC with different alpha
figure
t=1:length(E_rms_Proposed);

loglog(t,E_rms_timerever,'Color',Color(2,:),'Linewidth',LineW(4)); grid on; hold on;
loglog(t,E_rms_timerever2,'Color',Color(3,:),'Linewidth',LineW(4)); grid on; hold on;
loglog(t,E_rms_timerever3,'Color',Color(4,:),'Linewidth',LineW(4));
title('Convergence of time-reversal based ILC');
ylabel('RMS error [mm]');xlabel('iteration number');
axis([0, SimIter, 0, inf]);	
legend('\alpha = 0.5','\alpha = 1.2','\alpha = 1.6');
set(gcf,'position',[500 500 455 220])

%% Convergence Rate Acceleration

gamma_01 = 0.8;
gamma_02 = 0.9;
gamma_03 = 0.95;
gamma_04 = 0.99;

n = 5;
gamma1 = zeros(1,10);
gamma2 = zeros(1,10);
gamma3 = zeros(1,10);
gamma4 = zeros(1,10);

for p = 1:10

    gamma1(p) = (gamma_01)^((2^p)*(n));

end

for p = 1:10

    gamma2(p) = (gamma_02)^((2^p)*(n));

end

for p = 1:10

    gamma3(p) = (gamma_03)^((2^p)*(n));

end

for p = 1:10

    gamma4(p) = (gamma_04)^((2^p)*(n));

end

figure

p = 1:10;
plot(p,gamma1,'.-','Color',Color(1,:),'MarkerSize',10); grid on; hold on;
plot(p,gamma2,'.-','Color',Color(2,:),'MarkerSize',10); grid on; hold on; 
plot(p,gamma3,'.-','Color',Color(3,:),'MarkerSize',10); grid on; hold on;
plot(p,gamma4,'.-','Color',Color(4,:),'MarkerSize',10); grid on; 
title('Convergence Rate for updating period \itn=5');
ylabel('Convergence Rate ( \gamma_{\itpn} )');xlabel('number of acceleration times (\itp)');
legend('\gamma_0=0.8','\gamma_0=0.9','\gamma_0=0.95','\gamma_0=0.99')
% set(legend,'location','best')
set(gcf,'position',[500 500 455 220])


%% Learning Gain of NLIIC
y1 = ones(1,41);
y = 0:0.05:2;

for k = 1:41

   if y(k)>1
    y(k)=1;
   else
    y(k)=1/2*(1-cos(pi*y(k)));   
   end
    
end

figure
k = 0:0.05:2;

plot(k,y1,'Color','r','Linewidth',2, 'Linestyle','--'); hold on;
plot(k,y,'Color',[0 0.5 1],'Linewidth',2); grid on
title('Learning Gain of MFIIC and NLIIC');
ylabel('Learning Gain');xlabel('$\frac{|Y_j|}{|R|}$','Interpreter','latex');
legend('MFIIC','NLIIC')
set(gcf,'position',[500 500 455 220])