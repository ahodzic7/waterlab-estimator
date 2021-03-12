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

max_d_p_WW = 5;
min_d_p_WW = 3;
d_p_WW = disturbance_flow*(max_d_p_WW-min_d_p_WW) + min_d_p_WW;

figure
plot(d_t1_WW(1:t_MPC:end))
hold on
plot(d_p_WW(1:t_MPC:end))

%% Create rain 

rand_gen = gamrnd(0.5,1.5,round(size(d_t1_WW,2)/10),1);
%rand_gen = randn()

d_rain_temp = smooth(smooth(rand_gen));

for i = 1:size(d_rain_temp,1)
    if d_rain_temp(i) < 0.4
        d_rain_temp(i) = 0;
        if i > 20
            for k = 1:10
                d_rain_temp(i-k) = 0;
            end
        end
    end
end

d_t1_rain = smooth(d_rain_temp);

figure
plot(d_t1_rain)

d_t1_rain_sim = resample(d_t1_rain,t_MPC,1);

%% Plot

d_t1 = d_t1_rain_sim(1:N)' + d_t1_WW(1:N);
d_p = d_p_WW(1:N);

figure
subplot(2,1,1)
plot(d_t1)

subplot(2,1,2)
plot(d_p)

%% Save data

save('.\generated_data\d_lab','d_t1','d_p')
