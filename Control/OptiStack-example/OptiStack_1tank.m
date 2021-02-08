clearvars; clc; close all

% ************ Change to own Casadi path ************
addpath('C:\Users\adish\OneDrive\Documents\MATLAB\CasADi')
% ***************************************************
import casadi.*

%% ============================================== Sim. setup ===================================
Hp = 24;                                % prediction horizon   
nS = 1;                                 % number of states
opti = casadi.Opti();                   % opti stack 
N = 96;                                 % number of simulation steps
warmStartEnabler = 1;                   % warmstart 
intMethod = 1;                          % integration method

%% ============================================ Constraint limits ==============================
U_ub   = 0.008;                         % input
U_lb   = 0;
X_ub   = 2.5*ones(1,Hp+1);              % state
X_lb   = 0*ones(1,Hp+1);
deltaU = [-0.005; 0.005]*ones(1,Hp);    % slew rate (inactive if set to high values)

%% ========================================= Optimization variables ============================
X  = opti.variable(nS,Hp+1);            % state - volume 
U  = opti.variable(nS,Hp);              % input - pumpflow 
S  = opti.variable(nS,Hp+1);            % slack - overflow volume

%% ========================================= Optimization parameters ===========================
D  = opti.parameter(nS,Hp);             % disturbance - rain inflow
X0 = opti.parameter(nS);                % initial state - volume
T  = opti.parameter(nS);                % MPC model sampling time

%% =========================================== Simulation variables  ===========================
X_sim  = zeros(nS,N); 
U_sim  = casadi.DM.zeros(nS,N);
S_sim  = casadi.DM.zeros(nS,N);

%% =========================================== Objective =======================================
objective = sumsqr(S) + 100*sumsqr(U) + 0.0001*sumsqr(X);
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
    opti.subject_to(X_lb(k,:)+ S(k,:)<=X(k,:)<=X_ub(k,:)+ S(k,:));          % Soft constraint on state - volume 
    opti.subject_to(S(k,:)>=zeros(1,Hp+1));                                 % Slack variable is always positive - Vof >= 0
end
opti.subject_to(U_lb <=U<= U_ub);                                           % Bounded input  

for i=1:1:Hp                             
   opti.subject_to(deltaU(1,i) <= (U(1,i) - U(1,i-1)) <= deltaU(2,i));      % bounded slew rate
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
    OCP = opti.to_function('OCP',{X0,D,opti.lam_g,opti.x,T},{U(:,1),S(:,1),opti.lam_g,opti.x},{'x0','d','lam_g','x_init','dt'},{'u_opt','s_opt','lam_g','x_init'});
elseif warmStartEnabler == 0
    % Parametrized Open Loop Control problem without WARM START 
    OCP = opti.to_function('OCP',{X0,D,T},{U(:,1),S(:,1)},{'x0','d','dt'},{'u_opt','s_opt'});
end
  
%% ================================= MPC closed-loop sim. ==================================
% random disturbance signal
disturbance = (1/360)*[1,0,2,3,5,1,0,0,0,6,4,2,0,0,2,0,2,5,3,0,0,1,1,2,0,0,0,0,0,0,0,0,0,0,0,2,0,0,2,0,2,5,3,0,0,1,1,2,1,0,2,3,5,1,0,0,0,6,...
     4,2,0,0,2,0,2,5,3,0,0,1,1,2,0,0,0,0,0,0,0,0,0,0,0,2,0,0,2,0,2,5,3,0,0,1,1,2,1,0,2,3,5,1,0,0,0,6,4,2,0,0,2,0,2,5,3,0,0,1,1,2,0,0,0,0,0,...
     0,0,0,0,0,0,2,0,0,2,0,2,5,3,0,0,1,1,2,1,0,2,3,5,1,0,0,0,6,4,2,0,0,2,0,2,5,3,0,0,1,1,2,0,0,0,0,0,0,0,0,0,0,0,2,0,0,2,0,2,5,3,0,0,1,1,2];

X_sim(:,1) = 1;                                                             % init. state value     
lam_g = 0;                                                                  % init. multiplier for WARM START
x_init = 1;                                                                 % prev. state for WARM START
dt_sim = 600;                                                               % sampling time [s]

tic
disp('MPC running with Casadi')
for i = 1:1:N
    
    if warmStartEnabler == 1
    % Parametrized Open Loop Control problem with WARM START
    [U_sim(:,i), S_sim(:,i), lam_g, x_init] = (OCP(X_sim(:,i), disturbance(:,i:i+Hp-1), lam_g, x_init, dt_sim));
    elseif warmStartEnabler == 0
    % Parametrized Open Loop Control problem without WARM START 
    [U_sim(:,i), S_sim(:,i)] = (OCP(X_sim(:,i), disturbance(:,i:i+Hp-1), dt_sim));
    end
    % Simulate dynamics
    X_sim(:,i+1) = full(F_integral(X_sim(:,i), U_sim(:,i), disturbance(:,i), dt_sim ));        
    
end
toc

% Retrieve simulation variables
U_sim = full(U_sim);
S_sim = full(S_sim);
lam_g = full(lam_g);

%% ============================================= Debug ======================================
%
%% ========================================== Post-process ===================================
figure
subplot(2,1,1)
stairs(U_sim(1,:),'blue')
hold on
plot(disturbance(1,1:N),'black')
hold on
plot(U_ub(1)*ones(N,1),'color',[0 0.7 0],'linestyle','--')
xlim([1,N+1])
leg = legend('Input','Disturbance','Input limit');
set(leg,'Interpreter','latex');
%ylim([0, max(U_ub)])
%
subplot(2,1,2)
plot(X_sim(1,:),'red')
hold on
plot(S_sim(1,:),'magenta')
hold on
plot(X_ub(1)*ones(N,1),'color',[0 0.7 0],'linestyle','--')
xlim([1,N+1])
leg = legend('State','Slack','State limit');
set(leg,'Interpreter','latex');
