clearvars, clc, clear path

N = 5000;%2*2600;%2600;                                                                          % length of simulation (dependent on the length of disturbance data)

controlType = 2;                                                                         % switch between on/off and MPC

%% ============================================ Control setup ======================================
specifications;
%dataLoad;                                                                               % Load when comparing lab results

%% ===================================  Build dynamics & optimization  =============================
simulator_builder;                                                                       % build simulator dynamics
if controlType == 2
    MPC_builder_PDE;
end

%% =====================================  Initial conditions  ======================================
X_sim(1,1) = 3.3;%x(1,1);                                                                 % init. tank1 state [m^3]
X_sim(2,1) = 3.8;%x(2,1);                                                                 % init. tank2 state [m^3]                                                                     % warm start - Lagrange multiplier initializer
X_sim(Nxt+1:Nxt+Nxp,1) = 0.001;%x(Nxt+1:Nxt+Nxp,1);                                       % init. pipe states [m]
dt_sim = 0.5*t_resample/60;                                                               % sampling time [s]       

lam_g = 0;                                                                                % warm start - Lagrange multiplier initializer
x_init = 0.001;  

X_ref_design;                                                                             % Varying reference

%% ============================================  Forecasts  ========================================
load('D_sim')                                                                            % for testing reference tracking
%load('D_sim_mod')                                                                         % intensive rain events
%D_sim = D_sim_mod;

% Plot in real time
PlotType = 2;
if PlotType == 2
    U_opt = zeros(2,1);
    
    figure()
    subplot(2,2,1)
    x_plot{1} = plot(X_sim(1,:)');
    hold on
    plot(X_ref_sim(1,1:20:end))
    subplot(2,2,2)
    xpl{2} = plot(X_sim(2,:)');
    hold on
    plot(X_ref_sim(2,1:20:end))
    subplot(2,2,3)
    upl{1} = plot(U_opt(1,:)');
    subplot(2,2,4)
    upl{2} = plot(U_opt(2,:)');
end

%% ==============================================  Simulate  ======================================

disp('Simulator running')
tic

t_step = 20;

for i = 1:1:N                                                       
      
    if controlType == 1
        onoff_control;
    elseif controlType == 2    
        [U_MPC,S_MPC,Y_MPC,lam_g,x_init] = OCP(X_sim(:,i), D_sim(:,(i)*(t_step)-(t_step-1):t_step:(i-1)*t_step + (Hp)*t_step-(t_step-1)), P_sim, X_ref_sim(:,(i)*(t_step)-(t_step-2):t_step:(i-1)*t_step + (Hp)*t_step-(t_step-2)), lam_g, x_init, dt_sim);
        U_opt(:,i) = full(U_MPC);
        S_opt(:,i) = full(S_MPC);
    end
 
    % Dynamics simulator
    X_sim(:,i+1) = full(F_integral_sim(X_sim(:,i), U_opt(:,i), D_sim(:,1 + (i-1)*t_resample), P_sim, dt_sim ));
    
    progressbar(i/N)
    
    if PlotType == 2 && mod(i,Hp)==0
        drawPlots;
        i
        drawnow
    end
end
toc

%% Static plots
  plotResults;      



%% Save for GP training

%  x = X_sim(:,1:end-1);
%  u = U_opt;
%  d = D_sim(:,1:t_resample:size(U_opt,2)*t_resample);
% % 
% save('x','x')
% save('u','u')
% save('d','d')

%% Save for GP validation

% x_retrain2 = X_sim(:,1:end-1);
% u_retrain2 = U_opt;
% d_retrain2 = D_sim(:,1:t_resample:size(U_opt,2)*t_resample);
% % % % 
% save('x_retrain2','x_retrain2')
% save('u_retrain2','u_retrain2')
% save('d_retrain2','d_retrain2')
