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



SimIter       = 10000 ;

 
%% prepare plant model
Ts = 0.001;

P = zpk([0.9],[1 1],0.01,Ts);
C = tf([20 -19.9],[1 0],Ts);



GG = feedback(P*C,1);
[numGG, denGG] = tfdata(GG, 'v');   

figure
bode(GG);


%% prepare reference model

N = 2000;
fs = 1/Ts;			% sampling rate 
time = (0:N-1)/fs;		% time resolution in time-domain
r = 100*chirp(time,0,2,100)-100;		

r=[zeros(1,0.25*N) r zeros(1,0.25*N)]; % reference signal 


%% ILC to learn the reference signal: Proposed ILC

E_rms_Proposed=zeros(1,SimIter);
E_max_fft=zeros(1,SimIter);
R=fft(r);

n = 5; % update period
alpha = 0.08; % time-reversal learning gain
u = 1*r;  %initial control input u




disp('ILC Learning in Progress...');


for ii = 1:SimIter

    ii
    
    y = filter(numGG, denGG, u);
    Y=fft(y);
    
    
    e = r - y;
    ee=norm(e,2);
    RMS_error=sqrt((ee^2)/length(e));
    E_rms_Proposed(1,ii)=RMS_error;  
    E_max_fft(ii) = max(abs(e));
    
if (mod(ii,n))~=0  
   
    % time-reversal
    ee=filter(numGG, denGG, fliplr(e));
    Le = alpha*fliplr(ee);
    u=u+Le;

    
else
    
    % acceleration ILC
    
    U=fft(u); %control input in frequncy domain
    E=fft(e); %error signal in frequncy domain

    
    for k=1:length(R)            %proposed algorithm
     
     if  abs(R(k))< abs(E(k))
     
             
     else
         
     U(k)=U(k)+(U(k)/R(k))*E(k);  
    
     end   
    end
     
    
    u=ifft(U);

   
end



end

disp('ILC Learning Complete')
  

e_fft= r-y;    
ee=norm(e_fft,2);
e_RMS_fft=sqrt((ee^2)/length(e));

%% ILC to learn the reference signal: MFIIC

E_rms_MFIIC=zeros(1,SimIter);
E_max_MFIIC= zeros(1,SimIter);



R=fft(r);

u = r; % initial control input



disp('ILC Learning in Progress...');


for ii = 1:SimIter   
  
   ii
 
     
    y = filter(numGG, denGG, u);  

    
   
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
   %u=lowpass(u,100,fs);

    



end

disp('ILC Learning Complete')
  
e_MFIIC = r-y;
ee=norm(e_MFIIC,2);
e_RMS_MFIIC=sqrt((ee^2)/length(e));

%% ILC to learn the reference signal: Time-reversal based ILC

E_rms_timerever=zeros(1,SimIter);
E_max_timerever=zeros(1,SimIter);


u = r;  %initial control input u




disp('ILC Learning in Progress...');


for ii = 1:SimIter
    
    
    ii
    
    
    y = filter(numGG, denGG, u); % output signal

    
    e = r - y;
    ee=norm(e,2);
    RMS_error=sqrt((ee^2)/length(e));
    E_rms_timerever(1,ii)=RMS_error;  
    E_max_timerever(ii) = max(abs(e));
    
    
    ee=filter(numGG, denGG, fliplr(e));
    Le = alpha*fliplr( ee);
    u=u+Le;

  
   
end


e_timerev = r-y;
ee=norm(e_timerev,2);
e_RMS_timerev=sqrt((ee^2)/length(e));

disp('ILC Learning Complete')

%% NLIIC
E_rms_NLIIC=zeros(1,SimIter);
E_max_NLIIC = zeros(1,SimIter);

R=fft(r);

u = 1*r;  %initial control input u



disp('ILC Learning in Progress...');


for ii = 1:SimIter
    ii
    
    
   

    y = filter(numGG, denGG, u); % output signal

    
 
    
    e = r - y;
    ee=norm(e,2);
    RMS_error=sqrt((ee^2)/length(e));
    E_rms_NLIIC(1,ii)=RMS_error;  
    E_max_NLIIC(ii) = max(abs(e));

    Y=fft(y);
    U=fft(u); %control input in frequncy domain
    E=fft(e); %error signal in frequncy domain
    
    
    for k=1:length(R)            %proposed algorithm
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


 figure;
 t = 0:Ts:(length(r)-1)*Ts;
 cyan=[0 0.5 1];
 plot(t,r, 'Color',cyan,'Linewidth',1);hold on;
 xlabel('Time [s]');ylabel('Position [mm]');
 title('100 Hz Chirp Reference');
 grid on;
 set(gcf,'position',[500 500 455 220]);
 
 

figure;
t = 0:Ts:(length(r)-1)*Ts;
plot(t,e_NLIIC , 'Color',[0.95 0.81 0],'Linewidth',1);hold on;
plot(t,e_MFIIC , 'Color',[254  160  165]/255,'Linewidth',1);hold on;
plot(t,e_fft , 'Color',[0 0.71 1],'Linewidth',1);hold on;
xlabel('Time [s]');ylabel('Error [mm]');
legend('NLIIC','MFIIC','Proposed')
title('Tracking Error in the Steady State (no disturbance)');
grid on;
set(gcf,'position',[500 500 455 220]);


 
figure
t=1:length(E_rms_Proposed);
loglog(t,E_rms_timerever,'Color',Color(1,:),'Linewidth',LineW(1)); hold on;
loglog(t,E_rms_MFIIC,'Color',Color(2,:),'Linewidth',LineW(2)); hold on;
loglog(t,E_rms_NLIIC,'Color',Color(3,:),'Linewidth',LineW(3)); hold on;
loglog(t,E_rms_Proposed,'Color',Color(4,:),'Linewidth',LineW(4)); hold on;
grid on;
title('Convergence of the Tracking Error (no disturbance)'); ylabel('RMS error [mm]');xlabel('Iteration number');
axis([1, SimIter, 0, inf]);	
legend('Time-reversal','MFIIC','NLIIC','Proposed')
set(legend,'location','best')
set(gcf,'position',[500 500 455 220])

