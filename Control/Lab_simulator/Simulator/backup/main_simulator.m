clearvars, clc, clear path

N = 1000;                                                                       % length of simulation (dependent on the length of disturbance data)

%% ============================================ Control setup ======================================
specifications;

%% ===================================  Build dynamics & optimization  =============================
simulator_builder;                                                                 
MPC_builder;

%% Initial conditions for simulator
X_sim(1,1) = x(1,1);                                                                 % init. tank1 state [m^3]
X_sim(2,1) = x(2,1);                                                                 % init. tank2 state [m^3]                                                                     % warm start - Lagrange multiplier initializer
X_sim(Nxt+1:Nxt+Nxp,1) = x(Nxt+1:Nxt+Nxp,1);                                         % init. pipe states [m]
dt_sim = 0.5*t_resample/60;                                                          % sampling time [s]       

%% Initial conditions for MPC
lam_g = 0;                                                                           % warm start - Lagrange multiplier initializer
x_init = 0.001;  
X_ref_sim = [4;4];

%% Pre-computed inputs and disturbances

%U_sim(:,1:N) = u(:,1:N);
D_sim(:,1:N) = d(:,1:N);
U_sim(1,1:N) = u(1,1:N);

%% ==============================================  Simulate  ======================================

disp('Simulator running')

for i = 1:1:N                                                           

     onoff_control;
%     [U_MPC,S_MPC,Y_MPC,lam_g,x_init] = OCP(X_sim(:,i), D_sim(:,i:i+Hp-1), P_sim, lam_g, x_init, dt_sim);
%     
%     U_opt(:,i) = full(U_MPC);
    
    % Dynamics simulator
    X_sim(:,i+1) = full(F_integral_sim(X_sim(:,i), U_sim(:,i), D_sim(:,i), P_sim, dt_sim ));
    U_simulator(:,i) = full(U_sim(:,i)); 
    progressbar(i/N) 
    
end


%%

figure
ax(1) = subplot(3,2,1);
plot(D_sim(1,:)')
leg = legend('$d_{1}$');
set(leg,'Interpreter','latex');
title('Disturbances','interpreter','latex')
%
flow_model_est = P_sim(2)*(((X_sim(Nxt+ Nxp,:)).^(5/3)) ./ ((X_sim(Nxt+ Nxp,:) + P_sim(3)).^(2/3)));
ax(2) = subplot(3,2,2);
plot(y(1:N))
hold on
plot(flow_model_est)%+ test(1,1:end-1))
leg = legend('$d_{2}$');
set(leg,'Interpreter','latex');
title('Disturbances','interpreter','latex')
%
ax(3) = subplot(3,2,3);
plot(x(1,1:N))
hold on
plot(X_sim(1,:)')
leg = legend('$x_{t1}$','Limit');
set(leg,'Interpreter','latex');
title('Tank t1 state','interpreter','latex')
%
ax(4) = subplot(3,2,4);
plot(x(2,1:N))
hold on
plot(X_sim(2,:)')
leg = legend('$x_{t2}$','Limit');
set(leg,'Interpreter','latex');
title('Tank t2 state','interpreter','latex')
%
ax(5) = subplot(3,2,5);
plot(U_simulator(1,:)')
leg = legend('$u_{t1}$','Limit');
set(leg,'Interpreter','latex');
title('Pump 1','interpreter','latex')
%
ax(6) = subplot(3,2,6);
plot(u(2,1:N))
hold on
plot(U_simulator(2,:)')
leg = legend('$u_{t2}$','Limit');
set(leg,'Interpreter','latex');
title('Pump 2','interpreter','latex')

linkaxes(ax, 'x')

figure
title('Pipe states','interpreter','latex')
for i = 1:Nxp
    subplot(Nxp,1,i)
    plot(x(Nxt+i,1:N))
    hold on
    plot(X_sim(Nxt+i,:)');
    title(['Pipe state', num2str(i)],'interpreter','latex')
end
%%
% test = P_sim(2)*g(X_sim(Nxt+ Nxp,:),P_sim(3))
% 
% function y = g(z,p)    
%     y = ((z).^(5/3)) ./ ((z + p(3)).^(2/3));        
% end 


%% 