clearvars; clc; close all

% ************ Change to own Casadi path ************
addpath('C:\Users\Casper and Adis\Desktop\casadi-windows-matlabR2016a-v3.5.5')
% ***************************************************
import casadi.*
addpath('C:\Users\adish\OneDrive\Documents\MATLAB\CasADi')
addpath('OptiStack-example')
%% ============================================== Sim. setup ===================================
Hp = 24;                                % prediction horizon   
nS = 1;                                 % number of states
nU = 1;
opti = casadi.Opti();                   % opti stack 
N = 1500;                                % number of simulation steps
warmStartEnabler = 0;                   % warmstart 
intMethod = 2;                          % integration method

%% ============================================ Constraint limits ==============================
U_ub   = 0.03;                         % input
U_lb   = 0;
X_ub   = 5*ones(1,Hp+1);       % 3.5       % state
X_lb   = 2*ones(1,Hp+1);          %0.5
deltaU = [-3; 3]*ones(1,Hp);%[-0.03; 0.03]*ones(1,Hp);    % slew rate (inactive if set to high values)

%% ========================================= Optimization variables ============================
X  = opti.variable(nS,Hp+1);            % state - volume 
U  = opti.variable(nU,Hp);              % input - pumpflow 
delta_U = opti.variable(nU,Hp);
S_of  = opti.variable(nS,Hp+1);            % slack - overflow volume
S_uf  = opti.variable(nS,Hp+1);            % slack - for relaxing lower bound
Q_of = opti.variable(nU,Hp);

%% ========================================= Optimization parameters ===========================
D  = opti.parameter(nS,Hp);             % disturbance - rain inflow
X0 = opti.parameter(nS);                % initial state - volume
U0 = opti.parameter(nU);                % the previous control
T  = opti.parameter(nS);                % MPC model sampling time
R  = opti.parameter(nS);                % reference
sigma_D = opti.parameter(nS);           % disturbance variance - rain inflow
sigma_X = opti.parameter(nS,Hp+1);

%% =========================================== Simulation variables  ===========================
X_sim  = zeros(nS,N); 
X_sim_plot  = zeros(nS,N);
U_sim = casadi.DM.zeros(nS,N);
Q_of_sim = casadi.DM.zeros(nS,N);
S_of_sim  = casadi.DM.zeros(nS,N);
S_uf_sim  = casadi.DM.zeros(nS,N);
X_prediction = casadi.DM.zeros(nS,Hp+1);
U_sim_single_step = casadi.DM.zeros(nS,Hp);
Q_of_single_step = casadi.DM.zeros(nS,Hp);
S_of_sim_single_step  = casadi.DM.zeros(nS,Hp);
S_uf_sim_single_step  = casadi.DM.zeros(nS,Hp);


%% =========================================== Objective =======================================
Decreasing_cost = [(Hp):-1:1]*10000;
% P = 10000000000 + Decreasing_cost;
P = 1000000*ones(nS,Hp) + Decreasing_cost;
objective = 0.01*sumsqr(X-R) + 100*sum(S_of) + 100*sum(S_uf) + 40*sumsqr(U) + (10)*sum(sigma_X) + P*Q_of';
%sum(S) + 100*sumsqr(U) + 0.0001*sumsqr(X);
opti.minimize(objective); 

%% ============================================ Dynamics =======================================
dt = casadi.MX.sym('dt',1);             % sampling time 
x = casadi.MX.sym('x',nS);              % state
u = casadi.MX.sym('u',nS);              % input
d = casadi.MX.sym('d',nS);              % disturbance

% Runge Kutta constants
k1 = model(x, u, d);
k2 = model(x + dt / 2.0*k1, u , d);
k3 = model(x + dt / 2.0*k2, u , d);
k4 = model(x + dt*k3, u, d);

% Discrete dynamics
if intMethod == 1                       % Runge-Kutte 4th order
    xf = x + dt / 6.0 * (k1 + 2*k2 + 2*k3 + k4);
    F_integral = casadi.Function('F_RK4', {x, u, d, dt}, {xf}, {'x[k]', 'u[k]', 'd[k]', 'dt'}, {'x[k+1]'});
elseif intMethod == 2                   % Forward Euler
    xf = x + dt*model(x, u, d);
    F_integral = casadi.Function('F_EUL', {x, u, d, dt}, {xf}, {'x[k]', 'u[k]', 'd[k]', 'dt'}, {'x[k+1]'});
end
                                    
%% ==================================== Dynamics constraints ===============================
% Initial state                             
opti.subject_to(X(:,1)==X0);           

% Gap - closing constraint
for i=1:Hp                             
   opti.subject_to(X(:,i+1)==F_integral(X(:,i), U(:,i)+Q_of(:,i), D(:,i), T(:)));  
end

%% ==================================== Physical constraints ===============================
for k = 1:1:nS
    opti.subject_to(X_lb(k,:) - S_uf(k,:) <= X(k,:) <= X_ub(k,:)+ S_of(k,:) - sqrt(sigma_X(k,:))*norminv(0.95) )%- sqrt(sigma_X(k,:))*norminv(0.95)           % Soft constraint on state - volume 
    %opti.subject_to(-X(k,:) <= -X_lb(k,:) + sqrt(sigma_X(k,:))*norminv(0.95) - S(k,:));
    %X_lb(k,:) + S(k,:)<= X(k,:)
    
    
    opti.subject_to(sqrt(sigma_X(k,:))*norminv(0.95) >= S_of(k,:) >= zeros(1,Hp+1));                                 % Slack variable is always positive - Vof >= 0
    opti.subject_to(X_lb >= S_uf(k,:) >= zeros(1,Hp+1));  
    opti.subject_to(Q_of(k,:) >= zeros(1,Hp));                            % Overflow variable is always positive - Vof >= 0
end
opti.subject_to(U_lb <= U <= U_ub);                                           % Bounded input  

for i=1:1:Hp                             
   opti.subject_to(deltaU(1,i) <= (U(1,i) - U(1,i-1)) <= deltaU(2,i));        % bounded slew rate
   if i == 1
       opti.subject_to(delta_U(:,i)==U(:,i) - U0)
   else
       opti.subject_to(delta_U(:,i)==U(:,i) - U(:,i-1));
   end   
end

%% ================================= Uncertainty propagation ===============================
% Initial state                             
% opti.subject_to(sigma_X(:,1)==0.001);                                           % No uncertainty on the measured state     
% 
% Gap - closing constraint
% for i=1:Hp                             
%    opti.subject_to(sigma_X(:,i+1) == sigma_X(:,i) + sigma_D(:));  
% end

sigma_X_sim = zeros(nS,Hp+1);
sigma_X_sim(:,1) = 0; 
sigma_dist = 0.0045;
for i = 1:Hp
    sigma_X_sim(:,i+1) = sigma_X_sim(:,i) + sigma_dist;
end
tightened_constraint = X_ub(1)*ones(Hp+1,1)-sqrt(sigma_X_sim(1,:))'*norminv(0.95);

%% ====================================== Solver settings ==================================
%opti.set_initial(X, 1);                                                    % first guess
opti.set_initial(S_of, 0);
opti.set_initial(S_uf, 0);
opti.set_initial(U, 0);

opts = struct;
opts.ipopt.print_level = 1;
opts.print_time = true;
opts.expand = true;                                                         % makes function evaluations faster
%opts.ipopt.max_iter = 100;
opti.solver('ipopt',opts);

if warmStartEnabler == 1
    % Parametrized Open Loop Control problem with WARM START
    OCP = opti.to_function('OCP',{X0,U0,D,opti.lam_g,opti.x,T,R,sigma_D},{U(:,:),Q_of(:,:),S_of(:,:), S_uf(:,:),opti.lam_g,opti.x,sigma_X(:,:)},{'x0','u0','d','lam_g','x_init','dt','r','sigma_d'},{'u_opt','q_of','s_of_opt','s_uf_opt','lam_g','x_init','sigma_x'});
elseif warmStartEnabler == 0
    % Parametrized Open Loop Control problem without WARM START 
    OCP = opti.to_function('OCP',{X0,U0,D,T,R,sigma_D,sigma_X(:,:)},{U(:,:),Q_of(:,:),S_of(:,:),S_uf(:,:)},{'x0','u0','d','dt','r','sigma_d','sigma_x'},{'u_opt','q_of','s_of_opt','s_uf_opt'});
end
  
%% ================================= MPC closed-loop sim. ==================================
% random disturbance signal generation
rng('default');
rng(5)
t_dist = 0:(N+Hp);
smax =  1.5;
smin = -1;
dist_rescale = 0.8;
dist_offset = 0.017;

% stochastic disturbance
disturbance_mean = dist_rescale*0.05*abs(smooth(smooth(smooth(smin + (smax-smin)*rand(1,length(t_dist))))))';
disturbance_mean = resample(disturbance_mean,10,1);
disturbance_rand = smooth(smooth(disturbance_mean + dist_rescale*sigma_dist.*randn(size(disturbance_mean,2),1)'))';
disturbance_rand(disturbance_rand <= 0) = 0;

disturbance_mean = (disturbance_mean'+dist_offset)';
disturbance_rand = (disturbance_rand'+dist_offset)'
% 
figure
plot(disturbance_mean(1,1:N),'blue')
hold on
plot(disturbance_rand(1,1:N),'red--')

%%
X_sim(:,1) = 2.2;%0.8;                                                           % init. state value  
X_sim_plot(:,1) = X_sim(:,1);
lam_g = 0;                                                                  % init. multiplier for WARM START
x_init = 1;                                                                 % prev. state for WARM START
dt_sim = 1;                                                                % sampling time [s]
R_sim = 2;                                                                % reference
sigma_D_sim = sigma_dist*1;

tic
disp('MPC running with Casadi')
plotting = false;
if plotting
figure
end
U0_sim = 0;
for i = 1:1:N
    
    if warmStartEnabler == 1
    % Parametrized Open Loop Control problem with WARM START
    [U_sim_single_step, Q_of_single_step, S_of_sim_single_step,S_uf_sim_single_step, lam_g, x_init, sigma_X_sim(i,:)] = (OCP(X_sim(:,i),U0_sim, disturbance_mean(:,i:i+Hp-1), lam_g, x_init, dt_sim, R_sim, sigma_D_sim));
%     plot(i-1:1:Hp-2+i,full(U_sim_single_step))
%     hold on;
    elseif warmStartEnabler == 0
    % Parametrized Open Loop Control problem without WARM START 
    [U_sim_single_step, Q_of_single_step, S_of_sim_single_step,S_uf_sim_single_step] = (OCP(X_sim(:,i),U0_sim, disturbance_mean(:,i:i+Hp-1), dt_sim, R_sim, sigma_D_sim, sigma_X_sim));
    end
    U_sim(:,i) = U_sim_single_step(:,1);
    Q_of_sim(:,i) = Q_of_single_step(:,1);
    U0_sim = U_sim_single_step(:,1);
    S_of_sim(:,i) =  S_of_sim_single_step(:,1);
    S_uf_sim(:,i) =  S_uf_sim_single_step(:,1);
    
    % MPC prediction:
    X_prediction(:,1) = X_sim(:,i);
    for j=1:1:Hp
        X_prediction(:,j+1) = full(F_integral(X_prediction(:,j), U_sim_single_step(:,j) + Q_of_single_step(:,j), disturbance_mean(:,i+j-1), dt_sim ));
    end
    
    % Simulate dynamics
    %X_sim(:,i+1) = full(F_integral(X_sim(:,i), U_sim_single_step(:,1)+Q_of_single_step(:,1), disturbance_rand(:,i), dt_sim ));
    X_sim(:,i+1) = full(F_integral(X_sim(:,i), U_sim_single_step(:,1), disturbance_rand(:,i), dt_sim ));
    if X_sim(:,i+1) > X_ub(1) 
       X_sim(:,i+1) = X_ub(1);
    elseif X_sim(:,i+1) < 0
       X_sim(:,i+1) = 0;
    end
    X_sim_plot(:,i+1) = X_sim(:,i+1);
    if X_sim_plot(:,i+1) > X_ub(1) 
       X_sim_plot(:,i+1) = X_ub(1);
    elseif X_sim_plot(:,i+1) < 0
       X_sim_plot(:,i+1) = 0;
    end
    
    if plotting
        ax(1) = subplot(2,1,1);
        % Past state plot
        plot(0:1:i-1,full(X_sim_plot(:,1:i)),'r','LineWidth',1);
        hold on;
        % Prediction plot
        plot(i-1:1:Hp-1+i,full(X_prediction),'b','LineWidth',1.75);
        hold on;
        % Reference plot
        plot(0:1:Hp-1+i,ones(1,size(0:1:Hp-1+i,2))*R_sim,'k','LineWidth',1);
        hold on;
        % Plot slacks
        plot(0:1:i-1,full(S_of_sim(1,1:i)),'magenta');
        hold on;
        plot(i-1:1:Hp-1+i,full(S_of_sim_single_step),'magenta');
        % State constraint plot
        plot(0:1:i-1, 5*ones(1,size(0:1:i-1,2)),'g--','LineWidth',1);
        hold on;
        plot(i-1:1:Hp-1+i,tightened_constraint,'g--','LineWidth',1.75);
        hold off;
        leg = legend('Past level', 'Predicted level','Level Reference','Slack', 'Location','southeast');
        set(leg,'Interpreter','latex');
        ylim([0,5.1]);
        xlabel('k','Interpreter','latex');
        ylabel('$h_k$','Interpreter','latex');
        title('Prediction with chance constraint and high reference');
        if i <= 15
            xlim([0,37]);
        else
            xlim([i-15,i-15+37]);
        end
    end
    
%     if X_sim(:,i+1) > X_ub(:,1)
%         X_sim(:,i+1) = X_ub(:,1);
%     elseif X_sim(:,i+1) < X_lb(:,1)
%         X_sim(:,i+1) = X_lb(:,1);
%     end
    %progressbar(i/N) 
    OCP_results{i} = get_stats(OCP);
    % plot the inputs
    if plotting
        ax(2) = subplot(2,1,2);
        stairs(0:1:i-1,full(U_sim(:,1:i)),'b','LineWidth',1);
        hold on;
        stairs(i-1:1:Hp-2+i,full(U_sim_single_step(:,1:end)),'g','LineWidth',1.75);
        hold on;
        plot(i-1:1:i,[full(U_sim_single_step(:,1)),full(U_sim_single_step(:,1))],'r','LineWidth',2);
        hold on;
        plot(0:1:i-1,full(Q_of_sim(:,1:i)),'cyan','LineWidth',1)
        hold on;
        plot(i-1:1:Hp-2+i,full(Q_of_single_step(:,1:end)),'cyan--','LineWidth',1)
        hold off;
        leg = legend('Past action','Predicted action','Implemented action','Overflow','Location','northwest');
        set(leg,'Interpreter','latex');
        ylim([0,0.03]);
        xlabel('k','Interpreter','latex');
        ylabel('$u_k$','Interpreter','latex')
        if i <= 15
            xlim([0,37]);
        else
            xlim([i-15,i-15+37]);
        end
%         pause(1);
    end
%     if i == 598
%         1
%     end
end
toc
%%
% Retrieve simulation variables
U_sim = full(U_sim);
Q_of_sim = full(Q_of_sim);
S_of_sim = full(S_of_sim);
S_uf_sim = full(S_uf_sim);
lam_g = full(lam_g);
sigma_X_sim = full(sigma_X_sim);

%% ============================================= Debug ======================================
%
%% ========================================== Post-process ===================================
figure
ax(1) = subplot(2,1,1);
stairs(U_sim(1,:),'blue')
hold on
plot(disturbance_rand(1,1:N),'black')
hold on
plot(disturbance_mean(1,1:N),'black--')
hold on
plot(Q_of_sim(1,:),'cyan');
hold on;
plot(U_ub(1)*ones(N,1),'color',[0 0.7 0],'linestyle','--')
xlim([1,N+1])
leg = legend('Control','Disturbance','Dist. predicition','Overflow','Control limit');
xlabel('k','Interpreter','latex');
ylabel('$u_k$, $d_k$','Interpreter','latex');
title('Chance constrain SMPC with hight level reference');
set(leg,'Interpreter','latex');
%ylim([0, max(U_ub)])
ax(2) = subplot(2,1,2);
plot(X_sim_plot(1,:),'red')
hold on
% plot(S_of_sim(1,:),'magenta')
% hold on
% plot(S_uf_sim(1,:),'cyan')
% hold on
plot(X_ub(1)*ones(N,1),'color',[0 0.7 0],'linestyle','--')
hold on
% plot(R_sim*ones(N,1),'black')
% hold on
plot(X_lb(1)*ones(N,1),'color',[0 0.7 0],'linestyle','--')
xlim([1,N+1])
%ylim([1.9,5.1])
leg = legend('Level','Level limit');
%leg = legend('Tank level','Level Limit','Level Reference');
set(leg,'Interpreter','latex');
xlabel('k','Interpreter','latex');
ylabel('$h_k$','Interpreter','latex');
linkaxes(ax, 'x')

%% 
figure
plot(0:24,X_ub(1)*ones(Hp+1,1),'color',[0 0.7 0],'linestyle','--')
hold on
plot(0:24,X_ub(1)*ones(Hp+1,1)-sqrt(sigma_X_sim(1,:))'*norminv(0.95),'red')
grid on;
xlim([0,24]);
ylim([3.5,5.05]);
title('Chance constraint tightening');
xlabel('k','Interpreter','latex');
ylabel('$\bar{h}_k$','Interpreter','latex');
leg = legend('Physical bound','Tightend constraint');
set(leg,'Interpreter','latex');


%% test

% sigma propogation 
sigma_dist = 0.0045^2;
sigmaX = zeros(Hp,1);
sigmaX(1) = 0;
for k = 1:Hp
sigmaX(k+1) = sigmaX(k) + (sigma_dist);
end

tightening = sqrt(sigmaX)*norminv(0.95);
%%

fail_OCP_count = 0;
fail_OCP_where = 0;
T_mpc = 1;
len = N;
for i = T_mpc:T_mpc:len
if strcmp(OCP_results{i}.return_status,'Solve_Succeeded') == 0
fail_OCP_count = fail_OCP_count + 1;
fail_OCP_where(fail_OCP_count) = i;
end
end
fprintf('The solver found infeasibility: %d(%%) \n',int16((fail_OCP_count/len)*100))
