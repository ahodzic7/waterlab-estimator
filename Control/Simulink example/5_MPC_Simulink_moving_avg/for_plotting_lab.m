%% For Plotting Controller

clc;
clear;
close all;

load data_NMPC_MA
% load data_NMPC_MA_1_7_0_7

rho_fluid=997;
g=9.81;


set(0, 'DefaultLineLineWidth',1);
set(0, 'DefaultaxesLineWidth',1);
set(0, 'DefaultaxesFontSize',12);
set(0, 'DefaultTextFontSize',12);
set(0, 'DefaultAxesFontName','Times');


t=linspace(0,144,(T_min*60)+1);
t=t(1:T_min*60);

C_con_set=[0.7*ones(1,6),1.4*ones(1,14),0.7*ones(1,4)];
C=[];
for i=1:(T_min)/24
    C=horzcat(C,C_con_set);
end
C(:,end+1)=C(:,end);


t_Hp=0:144;

u1_MPC=out.u_1_MPC(1:T_min*60);
u2_MPC=out.u_2_MPC(1:T_min*60);

u1_act=out.u_1(1:T_min*60,1);
u2_act=out.u_2(1:T_min*60,1);


d2_set=d2(1:T_min*60,2);
d5_set=d5(1:T_min*60,2);

d2_act=out.d_1(1:T_min*60);
d5_act=out.d_2(1:T_min*60);

h4=out.h_4(1:T_min*60);

D_esti=out.D_esti(1:60:T_min*60+1);
p_esti=out.p_esti(1:60:T_min*60+1);

d2_esti=v1*D_esti;
d5_esti=v2*D_esti;

p4=h4*rho_fluid*g/1e5;


figure
subplot(2,1,1)
plot(t,u1_MPC,'b',t,u1_act,'g');
xlabel('Time [min]');ylabel('Flow [m^3/h]')
lgd=legend('NMPC set point','Actual flow');
title('Flow from the pumping station 1');
xlim([1 T_min]);
grid

subplot(2,1,2)
plot(t,u2_MPC,'b',t,u2_act,'g');
xlabel('Time [min]');ylabel('Flow [m^3/h]')
lgd=legend('NMPC set point','Actual flow');
title('Flow from the pumping station 2');
xlim([1 T_min]);
grid

figure
subplot(2,1,1)
plot(t,d2_set,'b',t,d2_act,'g');
xlabel('Time [min]');ylabel('Flow [m^3/h]')
lgd=legend('Demand set point','Actual demand');
title('Consumer 1 demand');
xlim([1 T_min]);
grid

subplot(2,1,2)
plot(t,d5_set,'b',t,d5_act,'g');
xlabel('Time [min]');ylabel('Flow [m^3/h]')
lgd=legend('Demand set point','Actual demand');
title('Consumer 2 demand');
xlim([1 T_min]);
grid



figure
subplot(2,1,1)
stairs(t_Hp,C,'r','LineWidth',2);
xlabel('Time [min]');ylabel('Price [DKK/kW min]')
lgd=legend('Price');
title('Price of electricity');
xlim([1 T_min]);
grid

subplot(2,1,2)
plot(t,h4,'g','LineWidth',2);
xlabel('Time [min]');ylabel('Level [m]')
lgd=legend('Tank Level: h_4');
title('Level of water in the tank');
xlim([1 T_min]);
grid

figure
plot(t,p4,'b',t_Hp,p_esti,'g--');
xlabel('Time [min]');ylabel('Pressure [bar]')
lgd=legend('Measurement with noise','Kalman filter estimate');
title('Comparison of actual tank pressure measurement from sensor and estimated pressure')
xlim([1 T_min]);
grid

figure
plot(t,(d2_act+d5_act),'b',t_Hp,-D_esti,'g--');
xlabel('Time [min]');ylabel('Flow [m^3/h]')
lgd=legend('Actual consumer demand','Estimated consumer demand');
title('Comparison of actual consumer demand and estimated consumer demand');
xlim([1 T_min]);
ylim([0 0.45]);
grid


figure
subplot(2,1,1)
plot_1=plot(t,d2_act,'b',t_Hp,-d2_esti,'g--');
xlabel('Time [min]');ylabel('Flow [m^3/h]')
lgd=legend('Actual d2 demand','Estimated d2 demand');
title('Comparison of actual d2 consumer demand and estimated d2 consumer demand');
xlim([0 T_min]);
ylim([0 0.26]);
grid

subplot(2,1,2)
plot_1=plot(t,d5_act,'b',t_Hp,-d5_esti,'g--');
xlabel('Time [min]');ylabel('Flow [m^3/h]')
lgd=legend('Actual d5 demand','Estimated d5 demand');
title('Comparison of actual d5 consumer demand and estimated d5 consumer demand');
xlim([0 T_min]);
ylim([0 0.18]);
grid


figure
subplot(2,1,1)
plot(t,p4,'b',t_Hp,p_esti,'g--');
xlabel('Time [min]');ylabel('Pressure [bar]')
lgd=legend('Measurement with noise','Kalman filter estimate');
title('Comparison of actual tank pressure measurement from sensor and estimated pressure')
xlim([1 T_min]);
grid

subplot(2,1,2)
plot(t,(d2_act+d5_act),'b',t_Hp,-D_esti,'g--');
xlabel('Time [min]');ylabel('Flow [m^3/h]')
lgd=legend('Actual consumer demand','Estimated consumer demand');
title('Comparison of actual consumer demand and estimated consumer demand');
xlim([1 T_min]);
ylim([0 0.45]);
grid
