clc;
clear all;
close all;

Ts=1;
Hp=24;
ts=1;
T_min=24*6;         % [min] Simulation time. Provide simulation time in multiple of 24 only.

% Simulation

T_sec=T_min*60;      % [s] Simulation time in seconds

N=(T_sec+(Hp*60))/ts;         % [] Simulation instances

% Generating demand curve for simulation. Number of data points to be for Hp(24 min) more than the
% simulation time. Data generated with sampling time of 1 sec

t=linspace(0,T_min+Hp,N);
d2=(0.3*(3*sin((t/(3.83))+5)-4*cos(2*t/(3.83))+8)*0.04)+0.02; % [m^3/s] -ve-> flow out of network
d5=(0.18*(3*sin((t/(3.83))+5)-4*cos(2*t/(3.83))+8)*0.04)+0.02; % [m^3/s] -ve-> flow out of network

time=0:1:length(d2);

d2=[time',[0;d2']];
d5=[time',[0;d5']];

% Generating cost of electricity
C_con_set=[0.7*ones(1,6*60),1.4*ones(1,14*60),0.7*ones(1,4*60)];
C_con_p=[];
for i=1:(T_min+Hp)/24
    C_con_p=horzcat(C_con_p,C_con_set);
end
time_con=0:1:length(C_con_p);
C_con_p=[time_con',[0;C_con_p']];

%% Load system parameters value

load('parameters_simulink.mat')