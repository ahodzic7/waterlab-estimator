clearvars; clc; close all

% ************ Change to own Casadi path ************
addpath('C:\Users\adish\OneDrive\Documents\MATLAB\CasADi')
% ***************************************************
import casadi.*

%% ============================================== Sim. setup ===================================
Hp = 24;                                % prediction horizon   
nS = 1;                                 % number of states
opti = casadi.Opti();                   % opti stack 
N = 500;                                % number of simulation steps
warmStartEnabler = 1;                   % warmstart 
intMethod = 1;                          % integration method

%% ============================================ Constraint limits ==============================
U_ub   = 0.03;                         % input
U_lb   = 0;
X_ub   = 5*ones(1,Hp+1);       % 3.5       % state
X_lb   = 2*ones(1,Hp+1);          %0.5
deltaU = [-0.03; 0.03]*ones(1,Hp);    % slew rate (inactive if set to high values)

%% ========================================= Optimization variables ============================
X  = opti.variable(nS,Hp+1);            % state - volume 
U  = opti.variable(nS,Hp);              % input - pumpflow 
S  = opti.variable(nS,Hp+1);            % slack - overflow volume
sigma_X = opti.variable(nS,Hp+1);       % state variance - volume

%% ========================================= Optimization parameters ===========================
D  = opti.parameter(nS,Hp);             % disturbance - rain inflow
X0 = opti.parameter(nS);                % initial state - volume
T  = opti.parameter(nS);                % MPC model sampling time
R  = opti.parameter(nS);                % reference
sigma_D = opti.parameter(nS);           % disturbance variance - rain inflow

%% =========================================== Simulation variables  ===========================
X_sim  = zeros(nS,N); 
U_sim = casadi.DM.zeros(nS,N);
S_sim  = casadi.DM.zeros(nS,N);
X_prediction = casadi.DM.zeros(nS,Hp+1);
U_sim_single_step  = casadi.DM.zeros(nS,Hp);
S_sim_single_step  = casadi.DM.zeros(nS,Hp);
sigma_X_sim  = casadi.DM.zeros(N,Hp+1);

%% =========================================== Objective =======================================
objective = 0.01*sumsqr(X-R) + 100*sum(S) + 40*sumsqr(U) + (10)*sum(sigma_X);
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
   opti.subject_to(X(:,i+1)==F_integral(X(:,i), U(:,i), D(:,i), T(:)));  
end

%% ==================================== Physical constraints ===============================
for k = 1:1:nS
    opti.subject_to(X_lb(k,:) - S(k,:) <= X(k,:) <= X_ub(k,:)+ S(k,:) - sqrt(sigma_X(k,:))*norminv(0.95) )%- sqrt(sigma_X(k,:))*norminv(0.95)           % Soft constraint on state - volume 
    %opti.subject_to(-X(k,:) <= -X_lb(k,:) + sqrt(sigma_X(k,:))*norminv(0.95) - S(k,:));
    %X_lb(k,:) + S(k,:)<= X(k,:)
    
    
    opti.subject_to(S(k,:) >= zeros(1,Hp+1));                                 % Slack variable is always positive - Vof >= 0
end
opti.subject_to(U_lb <= U <= U_ub);                                           % Bounded input  

for i=1:1:Hp                             
   opti.subject_to(deltaU(1,i) <= (U(1,i) - U(1,i-1)) <= deltaU(2,i));        % bounded slew rate
end

%% ================================= Uncertainty propagation ===============================
% Initial state                             
opti.subject_to(sigma_X(:,1)==0.001);                                           % No uncertainty on the measured state     

% Gap - closing constraint
for i=1:Hp                             
   opti.subject_to(sigma_X(:,i+1) == sigma_X(:,i) + sigma_D(:));  
end

%% ====================================== Solver settings ==================================
%opti.set_initial(X, 1);                                                    % first guess
opti.set_initial(S, 0);
opti.set_initial(U, 0);

opts = struct;
opts.ipopt.print_level = 1;
opts.print_time = true;
opts.expand = true;                                                         % makes function evaluations faster
%opts.ipopt.max_iter = 100;
opti.solver('ipopt',opts);

if warmStartEnabler == 1
    % Parametrized Open Loop Control problem with WARM START
    OCP = opti.to_function('OCP',{X0,D,opti.lam_g,opti.x,T,R,sigma_D},{U(:,:),S(:,:),opti.lam_g,opti.x,sigma_X(:,:)},{'x0','d','lam_g','x_init','dt','r','sigma_d'},{'u_opt','s_opt','lam_g','x_init','sigma_x'});
elseif warmStartEnabler == 0
    % Parametrized Open Loop Control problem without WARM START 
    OCP = opti.to_function('OCP',{X0,D,T,R,sigma_D},{U(:,:),S(:,:),sigma_X(:,:)},{'x0','d','dt','r','sigma_d'},{'u_opt','s_opt','sigma_x'});
end
  
%% ================================= MPC closed-loop sim. ==================================
% random disturbance signal generation
rng('default');
rng(4)
t_dist = 0:(N+Hp);
smax =  1.5;
smin = -1;

% stochastic disturbance
sigma_dist = 0.0065^2; %0.0045^2;
disturbance_mean = 0.05*abs(smooth(smooth(smooth(smin + (smax-smin)*rand(1,length(t_dist))))))';
disturbance_rand = disturbance_mean + sqrt(sigma_dist).*randn(N+Hp+1,1)';
disturbance_rand(disturbance_rand <= 0) = 0;

% figure
% plot(disturbance_mean,'blue')
% hold on
% plot(disturbance_rand,'red--')

%%
X_sim(:,1) = 2.2;%0.8;                                                           % init. state value     
lam_g = 0;                                                                  % init. multiplier for WARM START
x_init = 1;                                                                 % prev. state for WARM START
dt_sim = 60;                                                                % sampling time [s]
R_sim = 2.5;                                                                % reference
sigma_D_sim = sigma_dist*1;

tic
disp('MPC running with Casadi')
plotting = true;
figure
for i = 1:1:N
    
    if warmStartEnabler == 1
    % Parametrized Open Loop Control problem with WARM START
    [U_sim_single_step, S_sim_single_step, lam_g, x_init, sigma_X_sim(i,:)] = (OCP(X_sim(:,i), disturbance_mean(:,i:i+Hp-1), lam_g, x_init, dt_sim, R_sim, sigma_D_sim));
%     plot(i-1:1:Hp-2+i,full(U_sim_single_step))
%     hold on;
    U_sim(:,i) = U_sim_single_step(:,1);
    S_sim(:,i) =  S_sim_single_step(:,1);
    elseif warmStartEnabler == 0
    % Parametrized Open Loop Control problem without WARM START 
    [U_sim_single_step(:,:), S_sim_single_step(:,:), sigma_X_sim(i,:)] = (OCP(X_sim(:,i), disturbance_mean(:,i:i+Hp-1), dt_sim, R_sim, sigma_D_sim));
    end
    % MPC prediction:
    if plotting
        ax(1) = subplot(2,1,1);
        plot(0:1:i-1,full(X_sim(:,1:i)),'b');
        hold on;
    end
    X_prediction(:,1) = X_sim(:,i);
    for j=2:1:Hp
        X_prediction(:,j+1) = full(F_integral(X_prediction(:,j), U_sim_single_step(:,j), disturbance_mean(:,i+j-2), dt_sim ));
    end
    if plotting
        plot(i-1:1:Hp-1+i,full(X_prediction),'g');
        hold off;
        leg = legend('Past state', 'Predicted state');
        set(leg,'Interpreter','latex');
    end
    % Simulate dynamics
    X_sim(:,i+1) = full(F_integral(X_sim(:,i), U_sim_single_step(:,1), disturbance_rand(:,i), dt_sim ));        
    %progressbar(i/N) 
    OCP_results{i} = get_stats(OCP);
    % plot the inputs
    if plotting
        ax(2) = subplot(2,1,2);
        stairs(0:1:i-1,full(U_sim(:,1:i)),'b');
        hold on;
        plot(i-1:1:i,[full(U_sim_single_step(:,1)),full(U_sim_single_step(:,1))],'r');
        hold on;
        stairs(i:1:Hp-2+i,full(U_sim_single_step(:,2:end)),'g');
        hold off;
        leg = legend('Past input','Implemented input','Predicted action');
        set(leg,'Interpreter','latex');
        pause(1);
    end
end
toc
%%
% Retrieve simulation variables
U_sim = full(U_sim);
S_sim = full(S_sim);
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
plot(U_ub(1)*ones(N,1),'color',[0 0.7 0],'linestyle','--')
xlim([1,N+1])
leg = legend('Input','Disturbance','Dist. pred.','Input limit');
set(leg,'Interpreter','latex');
%ylim([0, max(U_ub)])
%
ax(2) = subplot(2,1,2);
plot(X_sim(1,:),'red')
hold on
plot(S_sim(1,:),'magenta')
hold on
plot(X_ub(1)*ones(N,1),'color',[0 0.7 0],'linestyle','--')
hold on
plot(R_sim*ones(N,1),'black')
hold on
plot(X_lb(1)*ones(N,1),'color',[0 0.7 0],'linestyle','--')
xlim([1,N+1])
leg = legend('State','Slack','Reference','State limit');
set(leg,'Interpreter','latex');

linkaxes(ax, 'x')

%% 
figure
plot(X_ub(1)*ones(Hp+1,1),'color',[0 0.7 0],'linestyle','--')
hold on
plot(X_ub(1)*ones(Hp+1,1)-sqrt(sigma_X_sim(1,:))'*norminv(0.95),'red')


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
