%% ================================ Init =============================== %% 
clear vars; clc; close all
addpath('C:\Users\adish\OneDrive\Documents\MATLAB\CasADi')
import casadi.*
opti = casadi.Opti();                   % opti stack 
pause on

%% ============================ System Info ============================ %% 
nS = 1;                                 % Number of System sates
nU = nS;                                % Number of System Inputs
dT = 1;                                 % Sampling time
initial_state = 2.5;
sigma_D = 0.0001;

% System model:
dt = casadi.MX.sym('dt',1);             % state
x = casadi.MX.sym('x',nS);              % state
u = casadi.MX.sym('u',nS);              % input
d = casadi.MX.sym('d',nS);              % disturbance
xf = x + dt*(d-u);                      % Change = disturbance - input
F_Euler = casadi.Function('EUL_mod', {x, u, d, dt}, {xf},...
     {'x[k]', 'u[k]', 'd[k]', 'dt'}, {'x[k+1]'});

%% ========================= Optimization Info ========================= %% 
Hp = 24;                                % prediction horizon
Hu = Hp;                                % Control horizon
% Paramteres (Known)
D  = opti.parameter(nS, Hp);            % Known disturbance - Rain inflow
X0 = opti.parameter(nS);                % Initial state - volume
sigma_X = opti.parameter(nS,Hp);

% Optimization variables (to be optimized)
X  = opti.variable(nS, Hp);             % System state - volume 
U  = opti.variable(nU, Hu);             % Control input - pumpflow 
S  = opti.variable(nU, Hu);             % 'slack' - overflow volume
S_ub = opti.variable(nS, Hu);

%% ========================= Objective Function ======================== %%
% Weights
Decreasing_cost = diag((nU*Hu):-1:1)*10000;
sum_vector = zeros(nU * Hu,1)+1;
P = eye(nU * Hu,nU * Hu) * 10000000000 + Decreasing_cost;
Q = eye(nS * Hp,nS * Hp) * 1000;
R = eye(nU * Hu,nU * Hu) * 100;

% Rearrange X and U
X_obj = vertcatComplete(X);
U_obj = vertcatComplete(U);
S_obj = vertcatComplete(S);

objective = X_obj'*Q*X_obj +U_obj'*R*U_obj+S_obj'*P*sum_vector + 10000*sum(S_ub);
opti.minimize(objective); 
%% ============================ Constraints =========================== %%
% Adding system dynamics - Where outflow is "Input + Slack" 
% Note this would change to two for loops if Hu != Hp
U_lb = 0;
U_ub = 0.03;

X_lb = 2;
X_ub = 5;
dU_lb = -0.015;
dU_ub = 0.015;

opti.subject_to(X(:,1)==X0);                    % Initial state constraints 

for i = 1:Hp-1                             
   opti.subject_to(X(:,i+1)==F_Euler(X(:,i), U(:,i) + S(:,i), D(:,i), dT));
   
   opti.subject_to(X_lb <= X(:,i));
   opti.subject_to(X <= X_ub + S_ub(:,i) - sqrt(sigma_X(:,i))*norminv(0.95));
end

for i = 1:Hp
    opti.subject_to(U_lb <= U <= U_ub);    
end

for i = 2:1:Hp                             
   opti.subject_to(dU_lb <= (U(:,i) - U(:,i-1)) <= dU_ub);      % bounded slew rate
end

for i = 1:1:Hp
    opti.subject_to(S(:,i)>=zeros(nU,1));      % Slack variable is always positive - Vof >= 0
    opti.subject_to(zeros(1,Hp) <= S_ub(:,i) <= sqrt(sigma_X(:,i))*norminv(0.95));                                 % Slack variable is always positive - Vof >= 0
end

%% ========================== Solver settings ========================== %%
                                               
opti.set_initial(S, 0);
opti.set_initial(U, 0);

opts = struct;
opts.ipopt.print_level = 1;
opts.print_time = true;
opts.expand = true;                     % makes function evaluations faster
%opts.ipopt.max_iter = 100;
opti.solver('ipopt',opts);

% Define optimization problem:
MPC = opti.to_function('MPC',{X0,D,sigma_X},{U,S,S_ub},{'x0','d','sigma_x'},{'u_opt','s_opt','S_ub_opt'});

%% ========================== Solver settings ========================== %%
N = 96;                                 % number of simulation steps
t_dist = 0:(N+Hp);
smax =  1.5;
smin = -1;
dist_rescale = 2;

% % Import forcast, add noise and set initial state
% forast_raw = readmatrix('test_disturbance.csv');
% dist_forcast = forast_raw(:,1:(N+Hp));
% dist = dist_forcast + normrnd(0,sigma_D,size(dist_forcast));
% dist(dist < 0) = 0;
% stochastic disturbance
rng('default');
rng(5)
sigma_dist = 0.0045^2; %0.0045^2;
dist_forcast = dist_rescale*0.15*abs(smooth(smooth(smooth(smin + (smax-smin)*rand(1,length(t_dist))))))';
dist = dist_forcast + sqrt(sigma_dist).*randn(N+Hp+1,1)';
dist(dist <= 0) = 0;

% Precompute sigma_X for chance constraint
sigma = zeros(nS,Hp);
sigma(:,1) = sigma_dist; 
for i = 1:Hp-1
    sigma(:,i+1) = sigma(:,i) + sigma_D;
end

X_sim = casadi.DM.zeros(nS, N+1); 
X_sim(:,1) = initial_state;
U_sim = casadi.DM.zeros(nU, N+1); 
S_sim = casadi.DM.zeros(nU, N+1); 
S_ub_sim = casadi.DM.zeros(nS, N+1);
X_predict = casadi.DM.zeros(nS, Hp); 

figure
%Run Closed loop mpc
for step = 1:1:N
    %Open loop predicition
    [U_out, S_out, S_ub_out] = (MPC(X_sim(:,step), dist_forcast(:,step:step+Hp-1),sigma));
    %Predict comming states:
    
    X_predict(:,1) = X_sim(step);
    for i = 1:Hp-1
        X_predict(:,i+1) = F_Euler(X_predict(:,i), U_out(:,i) + S_out(:,i), dist_forcast(:,step+i-1), dT);
    end
    
    %Advance simulation and save values
    X_sim(:,step+1) = F_Euler(X_sim(:,step), U_out(:,1), dist(:,step), dT);
    U_sim(:,step) = U_out(:,1);
    S_sim(:,step) = S_out(:,1);
    S_ub_sim(:,step) = S_ub_out(:,1);
    
    X_sim_num = full(X_sim);
    if X_sim_num(:,step+1) > X_ub 
       X_sim(:,step+1) = X_ub;
       X_sim_num(:,step+1) = X_ub;
    elseif X_sim_num(:,step+1) < X_lb
       X_sim(:,step+1) = X_lb;
       X_sim_num(:,step+1) = X_lb;
    end

    %Get numerical values
    U_out_num = full(U_out);
    S_out_num = full(S_out);
    S_ub_out_num = full(S_ub_out);
    X_predict_num = full(X_predict);
    U_sim_num = full(U_sim);
    S_sim_num = full(S_sim);
    S_ub_sim_num = full(S_ub_sim);
    
    
    clf
    subplot(2,1,1)
    % Time elapsed in simulation
    elapsed_time = 1:step;
    plot(elapsed_time, X_sim_num(1,1:step),'b')
    hold on
    %plot(elapsed_time, X_sim_num(2,1:step),'c')
    hold on
    % Current step
    plot(step, X_sim_num(1,step),'r*');
    hold on
    %plot(step, X_sim_num(2,step),'r*');
    hold on
    % Future predicitions
    future_time = elapsed_time(end):(elapsed_time(end)+Hp-1 );
    plot(future_time ,X_predict_num(1,:),'g');
    hold on
    %plot(future_time ,X_predict_num(2,:),'g');
    xlim([1,N+Hp])
    
    
    subplot(2,1,2)  
    plot(dist(1,:),'k')
    hold on
    plot(dist_forcast(1,:),'k--')
    hold on
    
    % Time elapsed in simulation
    elapsed_time = 1:step;
    stairs(elapsed_time, U_sim_num(1,1:step),'b')
    hold on
    plot(elapsed_time, S_sim_num(1,1:step),'g')
    hold on
    
    % Current step
    stairs(step, U_sim_num(1,step),'r*');
    hold on
    plot(step, S_sim_num(1,step),'r*');
    hold on

    % Future predicitions
    future_time = elapsed_time(end):(elapsed_time(end)+Hp-1 );
    stairs(future_time ,U_out_num,'b--');
    hold on
    plot(future_time ,S_out_num,'g--');
    hold on
    xlim([1,N+Hp])
    pause(0.25);
end
