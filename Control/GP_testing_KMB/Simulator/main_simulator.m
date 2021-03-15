clearvars, clc, clear path

N = 600;                                                                                % length of simulation (dependent on the length of disturbance data)

controlType = 3;                                                                         % switch between on/off, NMPC and GP-MPC

%% ============================================ Control setup ======================================
specifications;
%dataLoad;                                                                               % Load when comparing lab results

%% ===================================  Build dynamics & optimization  =============================
addpath('C:\Users\Casper and Adis\Desktop\casadi-windows-matlabR2016a-v3.5.5') 
simulator_builder;                                                                       % build simulator dynamics
if controlType == 2
    MPC_builder_PDE;
elseif controlType == 3
    MPC_builder_GP
end

%% Initial conditions for simulator
X_sim(1,1) = 4.3;%x(1,1);                                                                 % init. tank1 state [m^3]
X_sim(2,1) = 6.8;%x(2,1);                                                                 % init. tank2 state [m^3]                                                                     % warm start - Lagrange multiplier initializer
X_sim(Nxt+1:Nxt+Nxp,1) = 0.01;%x(Nxt+1:Nxt+Nxp,1);                                        % init. pipe states [m]
dt_sim = 0.5*t_resample/60;                                                               % sampling time [s]       

%% Initial conditions for MPC
lam_g = 0;                                                                                % warm start - Lagrange multiplier initializer
x_init = 0.01;  
sigma_X0 = zeros(Nx,Nx);

% Constant reference
%X_ref_sim = [3;3.5];

% Changing reference
X_ref_design;

%% Pre-computed inputs and disturbances

load('D_sim')

%% 
PlotType = 1;
if PlotType == 2
    U_opt = zeros(2,1);
    
    figure()
    subplot(2,2,1)
    x_plot{1} = plot(X_sim(1,:)');
    hold on
    plot(X_ref_sim(1,:))
    subplot(2,2,2)
    xpl{2} = plot(X_sim(2,:)');
    hold on
    plot(X_ref_sim(1,:))
    subplot(2,2,3)
    upl{1} = plot(U_opt(1,:)');
    subplot(2,2,4)
    upl{2} = plot(U_opt(2,:)');
end

%% ==============================================  Simulate  ======================================

disp('Simulator running')
tic
for i = 1:1:N                                                       
      
    if controlType == 1
        onoff_control;
    elseif controlType == 2    
        [U_MPC,S_MPC,Y_MPC,lam_g,x_init] = OCP(X_sim(:,i), D_sim(:,(i)*(20)-19:20:(i-1)*20 + (Hp)*20-19), P_sim, X_ref_sim(:,(i)*(20)-18:20:(i-1)*20 + (Hp)*20-18), lam_g, x_init, dt_sim);
        U_opt(:,i) = full(U_MPC);
    elseif controlType == 3
        [U_opt_Hp,mu_X_opt,sigma_X_opt,EPS,lam_g,x_init] = OCP(X_sim(:,i), D_sim(:,(i)*(20)-19:20:(i-1)*20 + (Hp)*20-19), sigma_X0, GP.z_train(:,1:M), GP.y_train(:,1:M), GP.sigma_f, GP.sigma_L, GP.sigma, lam_g, x_init, X_ref_sim(:,(i)*(20)-18:20:(i-1)*20 + (Hp)*20-18), GP.Beta);
        U_opt(:,i) = full(U_opt_Hp(:,1)); 
    end
 
    % Dynamics simulator
    X_sim(:,i+1) = full(F_integral_sim(X_sim(:,i), U_opt(:,i), D_sim(:,1 + (i-1)*t_resample), P_sim, dt_sim ));
    %U_simulator(:,i) = full(U_sim(:,i)); 
    progressbar(i/N)
    
    if PlotType == 2
        drawPlots;
        xlim([0,i])
        drawnow
    end
end
toc

%% Static plots
if PlotType == 1
  plotResults;      
end


%% Save for GP training

% x = X_sim(:,1:end-1);
% u = U_opt;
% d = D_sim(:,1:t_resample:end);
% 
% save('x','x')
% save('u','u')
% save('d','d')

%% Save for GP validation

% x_val = X_sim(:,1:end-1);
% u_val = U_opt;
% d_val = D_sim(:,1:t_resample:2600*t_resample);
% % 
% save('x_val','x_val')
% save('u_val','u_val')
% save('d_val','d_val')

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
