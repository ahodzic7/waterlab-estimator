clear;
% ************ Change to own Casadi path ************
addpath('C:\Users\Casper and Adis\Desktop\casadi-windows-matlabR2016a-v3.5.5')
% ***************************************************
import casadi.*


%% ============================================== MPC. setup ===================================
Hp = 24;                                % prediction horizon   
Hu = Hp;                                % control horizion
nS = 1;                                 % number of states
nT = 2;                                 % number of tanks
nP = 4;                                 % number of pipe sections
nU = 1;                                 % number of control inputs
nD = 2;
opti = casadi.Opti();                   % opti stack 
warmStartEnabler = 1;                   % warmstart for optimization
%% ============================================ Constraint limits ==============================
U_ub   = [8;10.5];                      % input bounds
U_lb   = [3;4.5];
dU_ub  = [3;3];
dU_lb  = [-3;-3];
Xt_ub  = 7.02;                          % state bounds tank
Xt_lb  = 1.50;
Xp_ub  = 0.5;                           % state bounds pipes                          
Xp_lb  = 0.00001;
% Combine into system bounds
X_ub   = [Xt_ub; Xp_ub*ones(1,nP); Xt_ub]'; 
X_lb   = [Xt_lb; Xp_lb*ones(1,nP); Xt_lb]'; 
%% ========================================= Optimization variables ============================
X  = opti.variable(nS,Hp+1);            % state - volume 
U  = opti.variable(nU,Hu);              % input - pumpflow 
S  = opti.variable(nS,Hp);              % slack - overflow volume

%% ========================================= Optimization parameters ===========================
D  = opti.parameter(nD,Hp);             % disturbance - rain inflow
X0 = opti.parameter(nS);                % initial state - level
T  = opti.parameter(1);                 % MPC model_level sampling time
Reference  = opti.parameter(nS);        % reference

%% ====================================== System parameters ====================================
p = [];
phi = [];

%% =========================================== Objective =======================================
% Weights
Decreasing_cost = diag((nU*Hu):-1:1)*1000000;
sum_vector = zeros(nU * Hu,1)+1;
P = eye(nU * Hu,nU * Hu) * 10000000 + Decreasing_cost;
Q = eye(nS * Hp+1,nS * Hp+1) * 100;
R = eye(nU * Hu,nU * Hu) * 10;

% Rearrange X and U
X_obj = vertcatComplete(X);
U_obj = vertcatComplete(U);
S_obj = vertcatComplete(S);

% Objective function
objective = (X_obj-Reference)'*Q*(X_obj-Reference) + U_obj'*R*U_obj+ S_obj'* P *sum_vector;
opti.minimize(objective);

%% ============================================ Dynamics =======================================

% function variables
dt = casadi.MX.sym('dt',1);             % sampling time 
x = casadi.MX.sym('x',nS);              % state
u = casadi.MX.sym('u',nS);              % input
d = casadi.MX.sym('d',nS);              % disturbance

% system matricies
A       = BuildA(nS, p, phi, dt);                                           % builds two tank topology with nS-2 pipe sections
B       = BuildB(nS, p, phi, dt);
Bd      = BuildBd(nS,3,p,phi,dt);                                           % allows d to enter in tank1 and in pipe section 2
Delta   = BuildDelta(nS, p, dt);
% function
system_dynamics = A*x + B*u + Bd*d + Delta;

% Discrete dynamics
F_System = casadi.Function('F_DW', {x, u, d, dt}, {system_dynamics}, {'x[k]', 'u[k]', 'd[k]', 'dt'}, {'x[k+1]'});

                                    
%% ======================================== Constraints ========================================
% Initial state                             
opti.subject_to(X(:,1)==X0);           

% Dynamic constraints
for i=1:Hp                             
   opti.subject_to(X(:,i+1)==F_system(X(:,i), U(:,i) + S(:,i), D(:,i), T));
   opti.subject_to(dU_lb <= (U(:,i) - U(:,i-1)) <= dU_ub);                  % bounded slew rate
   opti.subject_to(X_lb<=X(:,i)<=X_ub);                                     % level constraints 
end

for i = 1:1:nS
    opti.subject_to(S(i,:)>=zeros(1,Hp));                                   % slack variable is always positive - Vof >= 0
end

for i = 1:1:Hu
    opti.subject_to(U_lb <= U(:,i) <= U_ub);                                % bounded input  
end

%% ====================================== Solver settings ==================================
% opti.set_initial(X, 1);                                                    % first guess
opti.set_initial(S, 0);
opti.set_initial(U, U_lb);

opts = struct;
opts.ipopt.print_level = 1;
opts.print_time = true;
opts.expand = false;                                                         % makes function evaluations faster
%opts.ipopt.max_iter = 100;
opti.solver('ipopt',opts);

if warmStartEnabler == 1
    % Parametrized Open Loop Control problem with WARM START
    OCP = opti.to_function('OCP',{X0,D,opti.lam_g,opti.x,T,Reference},{U,S,opti.lam_g,opti.x},{'x0','d','lam_g','x_init','dt','ref'},{'u_opt','s_opt','lam_g','x_init'});
elseif warmStartEnabler == 0
    % Parametrized Open Loop Control problem without WARM START 
    OCP = opti.to_function('OCP',{X0,D,T,Reference},{U,S},{'x0','d','dt','ref'},{'u_opt','s_opt'});
end

load('C:\Git\waterlab-estimator\Control\Lab_Deterministic_MPC_tank1\D_sim.mat');
