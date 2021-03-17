clear;

%% Add paths

addpath('Stocastic_MPC\supporting_functions\matlab_sf')

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
U_ub   = [8;10.5]/60;                      % input bounds
U_lb   = [3;4.5]/60;
dU_ub  = [4.5;4.5]/60;
dU_lb  = [-4.5;-4.5]/60;
Xt_ub  = 7.02;                          % state bounds tank
Xt_lb  = 1.50;
Xp_ub  = 0.5;                           % state bounds pipes                          
Xp_lb  = -1;
% Combine into system bounds
X_ub   = [Xt_ub, Xp_ub*ones(1,nP), Xt_ub]'; 
X_lb   = [Xt_lb, Xp_lb*ones(1,nP), Xt_lb]'; 
%% ========================================= Optimization variables ============================
X  = opti.variable(nS,Hp+1);            % state - volume 
U  = opti.variable(nU,Hp);              % input - pumpflow 
deltaU = opti.variable(nU,Hp);
S  = opti.variable(nT,Hp);              % slack - overflow volume
S_ub = opti.variable(nT, Hu);           % slack - relaxing the chance constraint

%% ========================================= Optimization parameters ===========================
D  = opti.parameter(nD,Hp);             % disturbance - rain inflow
X0 = opti.parameter(nS);                % initial state - level
U0 = opti.parameter(nU);                % the previous control
T  = opti.parameter(1);                 % MPC model_level sampling time
Reference  = opti.parameter(nS);        % reference

sigma_X = opti.parameter(nT,Hp);

%% ====================================== System parameters ====================================
p = [0.0344584980456826,0.0864650413052119,0.00653614397630376,-0.00280609998794716,0.0550243659248174];
phi = [1/4.908738521234052,1/4.908738521234052];

%% =========================================== Objective =======================================
% Weights
Decreasing_cost = diag((nT*Hp):-1:1)*10000000;
sum_vector = zeros(nT * Hp,1)+1;
P = eye(nT * Hp,nT * Hp) * 1000000000 + Decreasing_cost;
Q = zeros(nS, nS);
Q(1,1) = 100;                                                               % cost of tank1 state
Q(6,6) = 100;                                                               % cost of tank2 state               
Q = kron(eye(Hp),Q);
R = eye(nU * Hp,nU * Hp) * 1;

% Rearrange X and U
X_obj = vertcatComplete( X(:,1:end-1) - Reference);
deltaU_obj = vertcatComplete(deltaU);
U_obj = vertcatComplete(U);
S_obj = vertcatComplete(S);

% Objective function
objective = X_obj'*Q*X_obj + S_obj'* P * sum_vector + deltaU_obj'*R*deltaU_obj + 10000*sum(sum(S_ub'));
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

% make struct to get when MPC is run
A_num       = casadi.Function('eval_A',{dt},{A},{'dt'},{'A'});
B_num       = casadi.Function('eval_B',{dt},{B},{'dt'},{'B'});
Bd_num      = casadi.Function('eval_Bd',{dt},{Bd},{'dt'},{'Bd'});
Delta_num   =casadi.Function('eval_Delta',{dt},{Delta},{'dt'},{'Delta'});
sys = struct('A', A_num, 'B', B_num, 'Bd',Bd_num,'Delta',Delta_num,'F_system',F_system,'X_lb', X_lb, 'X_ub' ,X_ub);


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
   opti.subject_to(X_lb(2:5)<=X(2:5,i)<=X_ub(2:5));  
end

for i = 1:1:nT
    opti.subject_to(S(i,:)>=zeros(1,Hp));                                   % slack variable is always positive - Vof >= 0
end

for i = 1:1:Hu
    opti.subject_to(U_lb <= U(:,i) <= U_ub);                                % bounded input  
end

%% ================================= Add Chance constraints ===============================
% Precompute sigma_X for chance constraint, Open Loop MPC:
var_x_prev = casadi.MX.sym('xvp',nS,nS);     
var_D = casadi.MX.sym('vd',nD,nD);              
var_model = casadi.MX.sym('vm',nS,nS); 

% function
var_x = A*var_x_prev*A' + Bd*var_D*Bd' + var_model;
% discrete dynamics
F_variance = casadi.Function('F_var', {var_x_prev, var_D, var_model,dt}, {var_x}, {'vx[k]', 'vd', 'vm','dt'}, {'vx[k+1]'});

% add constraints
for i = 1:1:Hp
   opti.subject_to(X_lb(1)<=X(1,i)<=X_ub(1) + S_ub(1,i) - sqrt(sigma_X(1,i))*norminv(0.95));
   opti.subject_to(X_lb(6)<=X(6,i)<=X_ub(6) + S_ub(2,i) - sqrt(sigma_X(2,i))*norminv(0.95));
   opti.subject_to(zeros(nT,1) <= S_ub(:,i) <= sqrt(sigma_X(:,i))*norminv(0.95));                                 % Slack variable is always positive - Vof >= 0
end



%% ====================================== Solver settings ==================================
% opti.set_initial(X, 1);                                                    % first guess
%opti.set_initial(S, 0);
%opti.set_inivar_x_prev = casadi.MX.sym('x',nS,nS);tial(U, U_lb);

opts = struct;
% opts.ipopt.print_level = 1;
% opts.print_time = true;
opts.expand = false;                                                         % makes function evaluations faster
%opts.ipopt.max_iter = 100;
opti.solver('ipopt',opts);

if warmStartEnabler == 1
    % Parametrized Open Loop Control problem with WARM START
    OCP = opti.to_function('OCP',{X0,U0,D,opti.lam_g,opti.x,T,Reference,sigma_X},{U,S,S_ub,opti.lam_g,opti.x},{'x0','u0','d','lam_g','x_init','dt','ref','sigma_x'},{'u_opt','s_opt','S_ub_opt','lam_g','x_init'});
elseif warmStartEnabler == 0
    % Parametrized Open Loop Control problem without WARM START 
    OCP = opti.to_function('OCP',{X0,U0,D,T,Reference,sigma_X},{U,S,S_ub},{'x0','u0','d','dt','ref','sigma_x'},{'u_opt','s_opt','S_ub_opt'});
end

load('.\Lab_Deterministic_MPC_full_system_Linear_DW\D_sim.mat');
load('.\Lab_Deterministic_MPC_full_system_Linear_DW\X_ref_sim.mat');
