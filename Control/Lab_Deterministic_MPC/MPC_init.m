clear;
% ************ Change to own Casadi path ************
addpath('C:\Users\Casper and Adis\Desktop\casadi-windows-matlabR2016a-v3.5.5')
% ***************************************************
import casadi.*

rand('seed', 1);

%% ============================================== Sim. setup ===================================
Hp = 24;                                % prediction horizon   
Hu = Hp;                                % control horizion
nS = 1;                                 % number of states
nU = 1;
opti = casadi.Opti();                   % opti stack 
warmStartEnabler = 1;                   % warmstart for optimization
intMethod = 2;                          % intergration method used in model
%% ============================================ Constraint limits ==============================
U_ub   = 10;                            % input
U_lb   = 4;
X_ub   = 6.70*ones(1,Hp+1);              % state
X_lb   = 1.50*ones(1,Hp+1);
deltaU = [-4; 4]*ones(1,Hp);    % slew rate (inactive if set to high values)

%% ========================================= Optimization variables ============================
X  = opti.variable(nS,Hp+1);            % state - volume 
U  = opti.variable(nS,Hu);              % input - pumpflow 
S  = opti.variable(nS,Hu);            % slack - overflow volume

%% ========================================= Optimization parameters ===========================
D  = opti.parameter(nS,Hu);             % disturbance - rain inflow
X0 = opti.parameter(nS);                % initial state - level
T  = opti.parameter(nS);                % MPC model_level sampling time
Reference  = opti.parameter(nS);        % reference

%% =========================================== Objective =======================================
% Weights
Decreasing_cost = diag((nU*Hu):-1:1)*10000;
sum_vector = zeros(nU * Hu,1)+1;
P = eye(nU * Hu,nU * Hu) * 100000000000000 + Decreasing_cost;
Q = eye(nS * Hp+1,nS * Hp+1) * 100000;
R = eye(nU * Hu,nU * Hu) * 10;

% Rearrange X and U
X_obj = vertcatComplete(X);
U_obj = vertcatComplete(U);
S_obj = vertcatComplete(S);

% Objective function
objective = (X_obj-Reference)'*Q*(X_obj-Reference) + U_obj'*R*U_obj+S_obj'*P*sum_vector;
opti.minimize(objective);

%% ============================================ Dynamics =======================================
dt = casadi.MX.sym('dt',1);             % sampling time 
x = casadi.MX.sym('x',nS);              % state
u = casadi.MX.sym('u',nS);              % input
d = casadi.MX.sym('d',nS);              % disturbance

% Runge Kutta constants
k1 = model_level(x, u, d);
k2 = model_level(x + dt / 2.0*k1, u , d);
k3 = model_level(x + dt / 2.0*k2, u , d);
k4 = model_level(x + dt*k3, u, d);

% Discrete dynamics
if intMethod == 1                       % Runge-Kutte 4th order
    xf = x + dt / 6.0 * (k1 + 2*k2 + 2*k3 + k4);
    F_integral = casadi.Function('F_RK4', {x, u, d, dt}, {xf}, {'x[k]', 'u[k]', 'd[k]', 'dt'}, {'x[k+1]'});
elseif intMethod == 2                   % Forward Euler
    xf = x + dt*model_level(x, u, d);
    F_integral = casadi.Function('F_EUL', {x, u, d, dt}, {xf}, {'x[k]', 'u[k]', 'd[k]', 'dt'}, {'x[k+1]'});
end
                                    
%% ==================================== Dynamics constraints ===============================
% Initial state                             
opti.subject_to(X(:,1)==X0);           

% Gap - closing constraint
for i=1:Hp                             
   opti.subject_to(X(:,i+1)==F_integral(X(:,i), U(:,i) + S(:,i), D(:,i), T(:)));  
end

%% ==================================== Physical constraints ===============================
for k = 1:1:nS
    opti.subject_to(X_lb(k,:)<=X(k,:)<=X_ub(k,:));                          % Soft constraint on state - volume 
    opti.subject_to(S(k,:)>=zeros(1,Hu));                                 % Slack variable is always positive - Vof >= 0
end

opti.subject_to(U_lb <=U<= U_ub);                                           % Bounded input  

for i=1:1:Hp                             
   opti.subject_to(deltaU(1,i) <= (U(1,i) - U(1,i-1)) <= deltaU(2,i));      % bounded slew rate
end

%% ====================================== Solver settings ==================================
% opti.set_initial(X, 1);                                                    % first guess
% opti.set_initial(S, 0);
% opti.set_initial(U, 0);

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

load('C:\Git\waterlab-estimator\Control\Lab_Deterministic_MPC\disturbance_flow.mat');
