clearvars, clc, clear path

N = 2000;                                                                       % length of simulation (dependent on the length of disturbance data)


%% ============================================ Control setup ======================================
specifications;
%load_disturbance;

%% ===================================  Build dynamics & optimization  =============================
simulator_builder;                                                                 
MPC_builder;

%%

%% Initial conditions for simulator
X_sim(1,1) = x(1,1);                                                                 % init. tank1 state [m^3]
X_sim(2,1) = x(2,1);                                                                 % init. tank2 state [m^3]                                                                     % warm start - Lagrange multiplier initializer
X_sim(Nxt+1:Nxt+Nxp,1) = x(Nxt+1:Nxt+Nxp,1);                                         % init. pipe states [m]
dt_sim = 0.5*t_resample/60;                                                          % sampling time [s]       

%% Initial conditions for MPC
lam_g = 0;                                                                           % warm start - Lagrange multiplier initializer
x_init = 0.01;  
%X_ref_sim = [3;3.5];
load('.\Lab_Deterministic_MPC_full_system_Linear_DW\X_ref_sim.mat');
X_ref_sim(:,1)

%% Pre-computed inputs and disturbances
%D_sim(:,1:N) = d(:,1:N);                                                             % for preliminary testing

load('D_sim')

% D_sim(1,:) = 1.1*d_t1(1,1:t_resample/2:(N+Hp+1)*t_resample/2);
% D_sim(2,:) = zeros(1,N+Hp+1);
% D_sim(3,:) = 0.7*d_p(1,1:t_resample/2:(N+Hp+1)*t_resample/2) + 0.9;

%% ==============================================  Simulate  ======================================

disp('Simulator running')
tic
for i = 1:1:N                                                       

%     onoff_control;
    [U_MPC,S_MPC,Y_MPC,lam_g,x_init] = OCP(X_sim(:,i), D_sim(:,(i)*(20)-19:20:(i-1)*20 + (Hp)*20-19), P_sim, X_ref_sim(:,20*i), lam_g, x_init, dt_sim);
    %%
    
    
    U_opt(:,i) = full(U_MPC);
    
    % Dynamics simulator
    X_sim(:,i+1) = full(F_integral_sim(X_sim(:,i), U_opt(:,i), D_sim(:,1 + (i-1)*t_resample), P_sim, dt_sim ));
    %U_simulator(:,i) = full(U_sim(:,i)); 
    progressbar(i/N) 
    
end
toc
%% test
% i = 400;
% [U_MPC,S_MPC,Y_MPC,lam_g,x_init] = OCP(X_sim(:,1), D_sim(:,i:i+Hp-1), P_sim, X_ref_sim, lam_g, x_init, dt_sim)

%%

plotEnabler = 1;
if plotEnabler == 1
figure
ax(1) = subplot(3,2,1);
plot(D_sim(1,1:t_resample:N*t_resample)')
% hold on
% plot(d(1,:))
leg = legend('$d_{1}$');
set(leg,'Interpreter','latex');
title('Disturbances','interpreter','latex')
%
flow_model_est = P_sim(2)*(((X_sim(Nxt+ Nxp,:)).^(5/3)) ./ ((X_sim(Nxt+ Nxp,:) + P_sim(3)).^(2/3)));
ax(2) = subplot(3,2,2);
plot(flow_model_est)%+ test(1,1:end-1))
% hold on
% plot(d(3,:) + u(1,:))
leg = legend('$d_{2}$');
set(leg,'Interpreter','latex');
title('Disturbances','interpreter','latex')
%
ax(3) = subplot(3,2,3);
plot(X_sim(1,:)')
hold on
plot(X_ref_sim(1)*ones(N,1),'black--')
hold on
plot(max_t1*ones(N,1),'red--')
hold on
plot(min_t1*ones(N,1),'red--')
% hold on
% plot(x(1,:))
leg = legend('$x_{t1}$','Reference');
set(leg,'Interpreter','latex');
title('Tank t1 state','interpreter','latex')
%
ax(4) = subplot(3,2,4);
plot(X_sim(2,:)')
hold on
plot(X_ref_sim(2)*ones(N,1),'black--')
hold on
plot(max_t2*ones(N,1),'red--')
hold on
plot(min_t2*ones(N,1),'red--')
% hold on
% plot(x(2,:))
leg = legend('$x_{t2}$','Reference');
set(leg,'Interpreter','latex');
title('Tank t2 state','interpreter','latex')
%
ax(5) = subplot(3,2,5);
plot(U_opt(1,:)')
hold on
plot(u1_on*ones(N,1),'red--')
hold on
plot(u1_off*ones(N,1),'red--')
% hold on
% plot(u(1,:),'red')
leg = legend('$u_{t1}$','Limit');
set(leg,'Interpreter','latex');
title('Pump 1','interpreter','latex')
%
ax(6) = subplot(3,2,6);
plot(U_opt(2,:)')
hold on
plot(u2_on*ones(N,1),'red--')
hold on
plot(u1_off*ones(N,1),'red--')
% hold on
% plot(u(2,:),'red')
leg = legend('$u_{t2}$','Limit');
set(leg,'Interpreter','latex');
title('Pump 2','interpreter','latex')

linkaxes(ax, 'x')

figure
title('Pipe states','interpreter','latex')
for i = 1:Nxp
    subplot(Nxp,1,i)
    plot(X_sim(Nxt+i,:)');
    title(['Pipe state', num2str(i)],'interpreter','latex')
end
end


%% Save disturbances that work

% D_sim_temp(1,:) = 1*d_t1(1,1:1:end);
% D_sim_temp(2,:) = zeros(1,size(d_t1,2)/1);
% D_sim_temp(3,:) = 0.8*d_p(1,1:1:end) + 2;
% % 
% for i = 1:size(d_t1,2)
% if D_sim_temp(1,i) <= 5
%     D_sim_temp(1,i) = 5;
%         for j = 1:20
%             D_sim_temp(1,i-j) = 5;
%         end
% end
% end
% % 
% 
%  D_sim = D_sim_temp;
%  
%  % include zeros 
%  N_zero = 1600;
%  indices = [300,3200,6000,9000,13000, 15000,18000, 21000, 25000,28000, 31000, 33500, 37000, 40000, 43000, 46000, 49000];
%  for i = 1:size(indices,2)
%  D_sim(3,indices(i):indices(i) + N_zero-1) = zeros(1,N_zero);
%  end
% % 
%  plot(D_sim(3,1:20:end))
% % 
% % % % 
%  save('D_sim','D_sim')
