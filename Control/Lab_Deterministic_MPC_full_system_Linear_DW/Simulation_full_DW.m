clear;
% ************ Change to own Casadi path ************
addpath('C:\Users\Casper and Adis\Desktop\casadi-windows-matlabR2016a-v3.5.5')
% ***************************************************
import casadi.*


%% ============================================== MPC. setup ===================================
Hp = 24;                                % prediction horizon   
Hu = Hp;                                % control horizion
nS = 6;                                 % number of states
nT = 2;                                 % number of tanks
nP = 4;                                 % number of pipe sections
nU = 2;                                 % number of control inputs
nD = 2;
opti = casadi.Opti();                   % opti stack 
warmStartEnabler = 0;                   % warmstart for optimization
%% ============================================ Constraint limits ==============================
U_ub   = [8;10.5];                      % input bounds
U_lb   = [3;4.5];
dU_ub  = [4.5;4.5];
dU_lb  = [-4.5;-4.5];
Xt_ub  = 7.02;                          % state bounds tank
Xt_lb  = 1.50;
Xp_ub  = 0.5;                           % state bounds pipes                          
Xp_lb  = 0.00001;
% Combine into system bounds
X_ub   = [Xt_ub, Xp_ub*ones(1,nP), Xt_ub]'; 
X_lb   = [Xt_lb, Xp_lb*ones(1,nP), Xt_lb]'; 
%% ========================================= Optimization variables ============================
X  = opti.variable(nS,Hp+1);            % state - volume 
U  = opti.variable(nU,Hp);              % input - pumpflow 
deltaU = opti.variable(nU,Hp);
S  = opti.variable(nT,Hp);              % slack - overflow volume

%% ========================================= Optimization parameters ===========================
D  = opti.parameter(nD,Hp);             % disturbance - rain inflow
X0 = opti.parameter(nS);                % initial state - level
U0 = opti.parameter(nU);                % the previous control
T  = opti.parameter(1);                 % MPC model_level sampling time
Reference  = opti.parameter(nS);     % reference

%% ====================================== System parameters ====================================
p = [0.0344584980456826,0.0864650413052119,0.00653614397630376,-0.00280609998794716,0.0550243659248174];
phi = [1/4.908738521234052,1/4.908738521234052];

%% =========================================== Objective =======================================
% Weights
Decreasing_cost = diag((nT*Hp):-1:1)*1000000;
sum_vector = zeros(nT * Hp,1)+1;
P = eye(nT * Hp,nT * Hp) * 10000000 + Decreasing_cost;
Q = zeros(nS, nS);
Q(1,1) = 1000;                                                               % cost of tank1 state
Q(6,6) = 1000;                                                               % cost of tank2 state               
Q = kron(eye(Hp),Q);
R = eye(nU * Hp,nU * Hp) * 10;

% Rearrange X and U
X_obj = vertcatComplete(X(:,2:end) - Reference);
deltaU_obj = vertcatComplete(deltaU);
S_obj = vertcatComplete(S);

% Objective function
objective = X_obj'*Q*X_obj + deltaU_obj'*R*deltaU_obj+ S_obj'* P * sum_vector;
opti.minimize(objective);

%% ============================================ Dynamics =======================================

% function variables
dt = casadi.MX.sym('dt',1);             % sampling time 
x = casadi.MX.sym('x',nS);              % state
u = casadi.MX.sym('u',nU);              % input
d = casadi.MX.sym('d',nD);              % disturbance

% system matricies
A       = BuildA(nS, p, phi, dt);                                           % builds two tank topology with nS-2 pipe sections
B       = BuildB(nS, p, phi, dt);
Bd      = BuildBd(nS,2,p,phi,dt);                                           % allows d to enter in tank1 and in pipe section 2
Delta   = BuildDelta(nS, p, dt);
% function
system_dynamics = A*x + B*u + Bd*d + Delta;

% Discrete dynamics
F_system = casadi.Function('F_DW', {x, u, d, dt}, {system_dynamics}, {'x[k]', 'u[k]', 'd[k]', 'dt'}, {'x[k+1]'});

                                    
%% ======================================== Constraints ========================================
% Initial state                             
opti.subject_to(X(:,1)==X0);           

% Defining control horizon.
for i=Hu+1:1:Hp
    opti.subject_to(U(:,i)==U(:,Hu))
end

% Dynamic constraints
for i=1:Hp                             
   opti.subject_to(X(:,i+1)==F_system(X(:,i), U(:,i) + S(:,i), D(:,i), T));
   if i == 1
       opti.subject_to(deltaU(:,i)==U(:,i) - U0)
   else
       opti.subject_to(deltaU(:,i)==U(:,i) - U(:,i-1));
   end
   opti.subject_to(dU_lb <= (U(:,i) - U(:,i-1)) <= dU_ub);                  % bounded slew rate
   opti.subject_to(X_lb<=X(:,i)<=X_ub);                                     % level constraints 
end

for i = 1:1:nT
    opti.subject_to(S(i,:)>=zeros(1,Hp));                                   % slack variable is always positive - Vof >= 0
end

for i = 1:1:Hu
    opti.subject_to(U_lb <= U(:,i) <= U_ub);                                % bounded input  
end

%% ====================================== Solver settings ==================================
% opti.set_initial(X, 1);                                                    % first guess
%opti.set_initial(S, 0);
%opti.set_initial(U, U_lb);

opts = struct;
opts.ipopt.print_level = 0;
opts.print_time = false;
opts.expand = false;                                                         % makes function evaluations faster
%opts.ipopt.max_iter = 100;
opti.solver('ipopt',opts);

if warmStartEnabler == 1
    % Parametrized Open Loop Control problem with WARM START
    OCP = opti.to_function('OCP',{X0,U0,D,opti.lam_g,opti.x,T,Reference},{U,S,opti.lam_g,opti.x},{'x0','u0','d','lam_g','x_init','dt','ref'},{'u_opt','s_opt','lam_g','x_init'});
elseif warmStartEnabler == 0
    % Parametrized Open Loop Control problem without WARM START 
    OCP = opti.to_function('OCP',{X0,U0,D,T,Reference},{U,S},{'x0','u0','d','dt','ref'},{'u_opt','s_opt'});
end

load('C:\Git\waterlab-estimator\Control\Lab_Deterministic_MPC_full_system_Linear_DW\D_sim.mat');

%% ====================================== Simulation ==================================


%Prep for sim and plot
N=100;                                                                     % number of simulation steps
X_sim = casadi.DM.zeros(nS, N+1); 
X_sim(:,1) = [4.3 0.01 0.01 0.01 0.01 6.8];
X_sim_num = full(X_sim);
X_predict = casadi.DM.zeros(nS, Hp+1);
reference = X_sim(:,1);

U_sim = casadi.DM.zeros(nU, N+1);
U_sim(:,1) = U_lb;
U_sim_num = full(U_sim);

S_sim = casadi.DM.zeros(nU, N+1); 
S_ub_sim = casadi.DM.zeros(nS, N+1);

%Set up sim disturbance:
disturbance = D_sim(1:2:3,1:N+Hp);
dist_forcast = disturbance + normrnd(0,0.1,size(disturbance));


%Variables for warmstart
lam_g = 1;
x_init = 0.001;

%Run Closed loop mpc
for step = 1:1:N
    %Open loop predicition
    if warmStartEnabler == 1
        % Parametrized Open Loop Control problem with WARM START
        [u , S, lam_g, x_init] = OCP(X_sim(:,step),U_sim(:,step),dist_forcast(:,step:1:step+Hp-1), lam_g, x_init, 0.5/60 ,reference);
    elseif warmStartEnabler == 0
        % Parametrized Open Loop Control problem without WARM START 
        [u , S] = (OCP(X_sim(:,step),U_sim(:,step),dist_forcast(:,step:1:step+Hp-1), 0.5/60, reference));
    end
    
    %Predict comming states:
    X_predict(:,1) = X_sim(:,step);
    for i = 1:Hp
        X_predict(:,i+1) = F_system(X_predict(:,i), u(:,1) + S(:,i), dist_forcast(:,step+i-1), 0.5/60);
    end
    
    %Get numerical value
    U_out_num = full(u);
    S_out_num = full(S);
    X_predict_num = full(X_predict);
    U_sim_num = full(U_sim);
    S_sim_num = full(S_sim);
   
    plot_MPC;
    
    %Advance simulation and save values
    X_sim(:,step+1) = F_system(X_sim(:,step), u(:,1), disturbance(:,step), 0.5);
    U_sim(:,step+1) = u(:,1);
    S_sim(:,step+1) = S(:,1);
    
    X_sim_num = full(X_sim);
    if X_sim_num(:,step+1) > X_ub 
       X_sim(:,step+1) = X_ub;
       X_sim_num(:,step+1) = X_ub;
    elseif X_sim_num(:,step+1) < X_lb
       X_sim(:,step+1) = X_lb;
       X_sim_num(:,step+1) = X_lb;
    end
end
