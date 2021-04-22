clear all; clc;
load('.\SMPC_realistic_disturbance\D_sim_ens');

%% ===== Calculating variance of disturbances =====
D_sim_ens = D_sim_ens/60;
Hp = 24;
ensambles_T1 = [D_sim_ens(1:10,:)];
mean_disturbance_T1 = mean(ensambles_T1);
variance_disturbance_T1 = var(ensambles_T1);
variance_prediction_T1 = zeros(1,size(D_sim_ens,2)-Hp);
for i = 1:size(D_sim_ens,2)-Hp
    variance_prediction_T1(i) = mean(variance_disturbance_T1(i:i+Hp));
end

ensambles_pipe = [D_sim_ens(21:30,:)];
mean_disturbance_pipe = mean(ensambles_pipe);
variance_disturbance_pipe = var(ensambles_pipe);
variance_prediction_pipe = zeros(1,size(D_sim_ens,2)-Hp);
for i = 1:size(D_sim_ens,2)-Hp
    variance_prediction_pipe(i) = mean(variance_disturbance_pipe(i:i+Hp));
end

mean_disturbance = [mean_disturbance_T1;D_sim_ens(11,:);mean_disturbance_pipe]*60;
average_dist_variance_Hp = [variance_prediction_T1; variance_prediction_pipe];

save('SMPC_realistic_disturbance/mean_disturbance.mat','mean_disturbance');
save('SMPC_realistic_disturbance/average_dist_variance_Hp.mat','average_dist_variance_Hp');
%%
figure
for i = 1:10
plot(ensambles_pipe(i,:),'--');
hold on;
end
plot(mean_disturbance_T1);
hold on;
plot(variance_disturbance_T1);
hold on;
plot(variance_prediction_T1);
legend('Realization 1','Realization 2', 'Mean disturbance', 'Dist Variance','Dist prediction avrg. variance');
figure
plot(mean_disturbance(1,:));
hold on;
plot(mean_disturbance(3,:));

figure
plot(average_dist_variance_Hp(1,:));
hold on;
plot(average_dist_variance_Hp(2,:));

