clear all; clc;

addpath('data');
load('.\data\disturbance_flow');

disturbance_flow = disturbance_flow/3 - 2;
t_MPC = 10;
N = 55000;

%% Create WW disturbance

max_d_t1_WW = 6;
min_d_t1_WW = 4;
d_t1_WW = disturbance_flow*(max_d_t1_WW-min_d_t1_WW) + min_d_t1_WW;

max_d_p_WW = 10;
min_d_p_WW = 4;
d_p_WW = disturbance_flow*(max_d_p_WW-min_d_p_WW) + min_d_p_WW;

% figure
% plot(d_t1_WW(1:t_MPC:end))
% hold on
% plot(d_p_WW(1:t_MPC:end))

%% Scenario generation d_t1_WW

var_t1_WW = 0.8;

d_t1_WW_s0 = d_t1_WW(1:t_MPC:end);
d_t1_WW_s1 = smooth(smooth(d_t1_WW(1:t_MPC:end) + (randn(size(d_t1_WW(1:t_MPC:end),2),1)*var_t1_WW)'))';
d_t1_WW_s2 = smooth(smooth(d_t1_WW(1:t_MPC:end) + (randn(size(d_t1_WW(1:t_MPC:end),2),1)*var_t1_WW)'))';
d_t1_WW_s3 = smooth(smooth(d_t1_WW(1:t_MPC:end) + (randn(size(d_t1_WW(1:t_MPC:end),2),1)*var_t1_WW)'))';
d_t1_WW_s4 = smooth(smooth(d_t1_WW(1:t_MPC:end) + (randn(size(d_t1_WW(1:t_MPC:end),2),1)*var_t1_WW)'))';
d_t1_WW_s5 = smooth(smooth(d_t1_WW(1:t_MPC:end) + (randn(size(d_t1_WW(1:t_MPC:end),2),1)*var_t1_WW)'))';
d_t1_WW_s6 = smooth(smooth(d_t1_WW(1:t_MPC:end) + (randn(size(d_t1_WW(1:t_MPC:end),2),1)*var_t1_WW)'))';
d_t1_WW_s7 = smooth(smooth(d_t1_WW(1:t_MPC:end) + (randn(size(d_t1_WW(1:t_MPC:end),2),1)*var_t1_WW)'))';

d_t1_WW_S = [d_t1_WW_s0; d_t1_WW_s1; d_t1_WW_s2; d_t1_WW_s3; d_t1_WW_s4; d_t1_WW_s5; d_t1_WW_s6; d_t1_WW_s7];

% upper limit calculation 
for i = 1:size(d_t1_WW_s1,2)
        d_t1_WW_UB(1,i) = max(d_t1_WW_S(:,i));
end
% lower limit calculation 
for i = 1:size(d_t1_WW_s1,2)
        d_t1_WW_LB(1,i) = min(d_t1_WW_S(:,i));
end
% mean calculation 
d_t1_WW_mean = mean(d_t1_WW_S,1);

figure
plot(d_t1_WW(1:t_MPC:end))
hold on
plot(d_t1_WW_s1)
hold on
plot(d_t1_WW_s2)
hold on
plot(d_t1_WW_s3)
hold on
plot(d_t1_WW_s4)
hold on
plot(d_t1_WW_s5)
hold on
jbfill([1:size(d_t1_WW_s1,2)],d_t1_WW_UB,d_t1_WW_LB,'black','none',1,0.2)
hold on
plot(d_t1_WW_mean,'black--','LineWidth',1.2)
grid on
xlim([0,1000])

%% Scenario generation d_p WW

var_p_WW = 2.5;

d_p_WW_s0 = d_p_WW(1:t_MPC:end);
d_p_WW_s1 = smooth(smooth(d_p_WW(1:t_MPC:end) + (randn(size(d_p_WW(1:t_MPC:end),2),1)*var_p_WW)'))';
d_p_WW_s2 = smooth(smooth(d_p_WW(1:t_MPC:end) + (randn(size(d_p_WW(1:t_MPC:end),2),1)*var_p_WW)'))';
d_p_WW_s3 = smooth(smooth(d_p_WW(1:t_MPC:end) + (randn(size(d_p_WW(1:t_MPC:end),2),1)*var_p_WW)'))';
d_p_WW_s4 = smooth(smooth(d_p_WW(1:t_MPC:end) + (randn(size(d_p_WW(1:t_MPC:end),2),1)*var_p_WW)'))';
d_p_WW_s5 = smooth(smooth(d_p_WW(1:t_MPC:end) + (randn(size(d_p_WW(1:t_MPC:end),2),1)*var_p_WW)'))';
d_p_WW_s6 = smooth(smooth(d_p_WW(1:t_MPC:end) + (randn(size(d_p_WW(1:t_MPC:end),2),1)*var_p_WW)'))';
d_p_WW_s7 = smooth(smooth(d_p_WW(1:t_MPC:end) + (randn(size(d_p_WW(1:t_MPC:end),2),1)*var_p_WW)'))';

d_p_WW_S = [d_p_WW_s0; d_p_WW_s1; d_p_WW_s2; d_p_WW_s3; d_p_WW_s4; d_p_WW_s5; d_p_WW_s6; d_p_WW_s7];

% upper limit calculation 
for i = 1:size(d_p_WW_s1,2)
        d_p_WW_UB(1,i) = max(d_p_WW_S(:,i));
end
% lower limit calculation 
for i = 1:size(d_p_WW_s1,2)
        d_p_WW_LB(1,i) = min(d_p_WW_S(:,i));
end
% mean calculation 
d_p_WW_mean = mean(d_p_WW_S,1);

figure
plot(d_p_WW(1:t_MPC:end))
hold on
plot(d_p_WW_s1)
hold on
plot(d_p_WW_s2)
hold on
plot(d_p_WW_s3)
hold on
plot(d_p_WW_s4)
hold on
plot(d_p_WW_s5)
hold on
jbfill([1:size(d_p_WW_s1,2)],d_p_WW_UB,d_p_WW_LB,'black','none',1,0.2)
hold on
plot(d_p_WW_mean,'black--','LineWidth',1.2)
grid on
xlim([0,1000])


%% Create rain for tank 1

a = 0.5;    % shape
b = 1.5;    % scale
rand_gen = gamrnd(a,b,round(size(d_t1_WW,2)/t_MPC),1);
rand_gen_s1 = gamrnd(a,b,round(size(d_t1_WW,2)/t_MPC),1);
rand_gen_s2 = gamrnd(a,b,round(size(d_t1_WW,2)/t_MPC),1);


d_t1_rain_temp = smooth(smooth(rand_gen));
d_t1_rain_s1_temp = smooth(smooth(rand_gen_s1));
d_t1_rain_s2_temp = smooth(smooth(rand_gen_s2));

for i = 1:size(d_t1_rain_temp,1)
    if d_t1_rain_temp(i) < 0.4
        d_t1_rain_temp(i) = 0;
        if i > 20
            for k = 1:20
                d_t1_rain_temp(i-k) = 0;
            end
        end
    end
    
    if d_t1_rain_temp(i) < 0.4
        d_t1_rain_s1_temp(i) = 0;
        if i > 20
            for k = 1:20
                d_t1_rain_s1_temp(i-k) = 0;
            end
        end
    end
    
    if d_t1_rain_temp(i) < 0.4
        d_t1_rain_s2_temp(i) = 0;
        if i > 20
            for k = 1:20
                d_t1_rain_s2_temp(i-k) = 0;
            end
        end
    end
end

d_t1_rain = smooth(d_t1_rain_temp)*2.5;
d_t1_rain_s1 = smooth(d_t1_rain_s1_temp)*2.5;
d_t1_rain_s2 = smooth(d_t1_rain_s2_temp)*2.5;

figure
plot(d_t1_rain)
hold on
plot(d_t1_rain_s1)
hold on
plot(d_t1_rain_s2)

d_t1_rain_sim = resample(d_t1_rain,t_MPC,1);

%% Create rain for pipe

d_p_rain_temp = smooth(smooth(rand_gen));

for i = 1:size(d_p_rain_temp,1)
    if d_p_rain_temp(i) < 0.4
        d_p_rain_temp(i) = 0;
        if i > 15
            for k = 1:15
                d_p_rain_temp(i-k) = 0;
            end
        end
    end
end

d_p_rain = smooth(d_p_rain_temp)*1.5;

figure
plot(d_p_rain)

d_p_rain_sim = resample(d_p_rain,t_MPC,1);


%% Plot

d_t1 = d_t1_rain_sim(1:N)' + d_t1_WW(1:N)*1.2;
d_p  = d_p_rain_sim(1:N)' + d_p_WW(1:N);

figure
subplot(2,1,1)
plot(d_t1)

subplot(2,1,2)
plot(d_p)

d_t2 = zeros(1,N);

%% Save data
D_sim_mod = resample([d_t1; d_t2; d_p]',4,1)';%[d_t1; d_t2; d_p];
%save('.\generated_data\D_sim_mod','D_sim_mod')
