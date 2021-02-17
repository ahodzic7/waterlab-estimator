%% ================================ Init =============================== %% 
clear vars; clc; close all
addpath('C:\Users\Casper\OneDrive - Aalborg Universitet\MATLAB\casadi-windows-matlabR2016a-v3.5.5')
import casadi.*
opti = casadi.Opti();                   % opti stack 
pause on

%% ============================ System Info ============================ %% 
nS = 1;                                 % Number of System sates
nU = nS;                                % Number of System Inputs
dT = 1;                                 % Sampling time
initial_state = 10;

% System model:
dt = casadi.MX.sym('dt',1);              % state
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
% Optimization variables (to be optimized)
X  = opti.variable(nS, Hp);             % System state - volume 
U  = opti.variable(nU, Hu);             % Control input - pumpflow 
S  = opti.variable(nU, Hu);             % 'slack' - overflow volume

%% ========================= Objective Function ======================== %%
% Weights
P = eye(nU * Hu,nU * Hu) * 100000;
Q = eye(nS * Hp,nS * Hp) * 1000;
R = eye(nU * Hu,nU * Hu) * 100;

% Rearrange X and U
X_obj = vertcatComplete(X);
U_obj = vertcatComplete(U);
S_obj = vertcatComplete(S);
objective = X_obj'*Q*X_obj +U_obj'*R*U_obj+S_obj'*P*S_obj;
opti.minimize(objective); 

%% ============================ Constraints =========================== %%
% Adding system dynamics - Where outflow is "Input + Slack" 
% Note this would change to two for loops if Hu != Hp
for i = 1:Hp-1                             
   opti.subject_to(X(:,i+1)==F_Euler(X(:,i), U(:,i) + S(:,i), D(:,i), dT));  
end

opti.subject_to(X(:,1)==X0);                    % Initial state constraint

X_lb = 0;
X_ub = 25;
dU_lb = -1;
dU_ub = 1;

for i = 1:Hp
    opti.subject_to(X_lb<=X(:,i)<=X_ub);        % Tank size constriant
end

for i = 2:1:Hp                             
   opti.subject_to(dU_lb <= (U(:,i) - U(:,i-1)) <= dU_ub);      % bounded slew rate
end

for i = 1:1:Hp
    opti.subject_to(S(:,i)>=zeros(nU,1));      % Slack variable is always positive - Vof >= 0
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
MPC = opti.to_function('MPC',{X0,D},{U,S},{'x0','d'},{'u_opt','s_opt'});

%% ========================== Solver settings ========================== %%
N = 96;                                 % number of simulation steps

% Import forcast, add noise and set initial state
forast_raw = readmatrix('test_disturbance.csv');
dist_forcast = forast_raw(:,1:(N+Hp));
dist = dist_forcast + normrnd(0,0.1,size(dist_forcast));
dist(dist < 0) = 0;

X_sim = casadi.DM.zeros(nS, N+1); 
X_sim(:,1) = initial_state;
U_sim = casadi.DM.zeros(nU, N+1); 
S_sim = casadi.DM.zeros(nU, N+1); 

X_predict = casadi.DM.zeros(nS, Hp); 

figure
%Run Closed loop mpc
for step = 1:1:N
    %Open loop predicition
    [U_out, S_out] = (MPC(X_sim(:,step), dist_forcast(:,step:step+Hp-1)));
    %Predict comming states:
    
    X_predict(:,1) = X_sim(step);
    for i = 1:Hp-1
        X_predict(:,i+1) = F_Euler(X_predict(:,i), U_out(:,i) + S_out(:,i), dist_forcast(:,step+i-1), dT);
    end
    
    %Advance simulation and save values
    X_sim(:,step+1) = F_Euler(X_sim(:,step), U_out(:,1) + S_out(:,1), dist(:,step), dT);
    U_sim(:,step) = U_out(:,1);
    S_sim(:,step) = S_out(:,1);
    
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
    X_predict_num = full(X_predict);
    U_sim_num = full(U_sim);
    S_sim_num = full(S_sim);
    
    
    clf
    subplot(2,1,1)
    % Time elapsed in simulation
    elapsed_time = 1:step;
    plot(elapsed_time, X_sim_num(1,1:step),'b')
    hold on
    % Current step
    plot(step, X_sim_num(1,step),'r*');
    hold on
    % Future predicitions
    future_time = elapsed_time(end):(elapsed_time(end)+Hp-1 );
    plot(future_time ,X_predict_num,'g');
    xlim([1,N+Hp])
    
    subplot(2,1,2)  
    plot(dist,'k--')
    hold on
    plot(dist_forcast,'k')
    hold on
    
    % Time elapsed in simulation
    elapsed_time = 1:step;
    plot(elapsed_time, U_sim_num(1,1:step),'b')
    hold on
    % Current step
    plot(step, U_sim_num(1,step),'r*');
    hold on
    % Future predicitions
    future_time = elapsed_time(end):(elapsed_time(end)+Hp-1 );
    plot(future_time ,U_out_num,'g');
    xlim([1,N+Hp])
    pause(0.25);
end
