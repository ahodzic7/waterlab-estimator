%% casadi_builder.m
% """
% Builds optimization problem
% """
import casadi.*
opti = casadi.Opti();                                       % Optimization problem

%% ============================================ Constraint bounds =============================
U_ub = [u1_on ; u2_on];                                     % Input bounds
U_lb = [u1_off ; u2_off];

Xt_ub = [max_t1 ; max_t2]*ones(1,Hp+1);                     % Tank state bounds
Xt_lb = [min_t1 ; min_t2]*ones(1,Hp+1);

Xp_ub = h_p_max*ones(1,Hp+1);                               % pipe state bounds
Xp_lb = h_p_min*ones(1,Hp+1);

%% ========================================= Optimization variables ============================
% multiple shooting -> X is an optimization variable
X    = opti.variable((Nxt+ Nxp),Hp+1);                      % state trajectory   
U    = opti.variable(Nxt,Hp);                               % control trajectory | integral control
S    = opti.variable(Nxt,Hp+1);                             % slack variable for tracting overflow

%% ========================================= Optimization parameters ===========================
D     = opti.parameter(ND,Hp);                              % disturbance trajectory | uncontrolled inflow 
P     = opti.parameter(NP,1);                               % pipe parameters
X0    = opti.parameter(Nxt+ Nxp);                           % initial state 
DT    = opti.parameter(1);                                  % MPC model sampling time 
X_ref = opti.parameter(Nxt,Hp); 

%% ========================================= Output variables ============================
Y     = P(2)*g(X(Nxt+ Nxp,:),P(3));

%% =========================================== Objective function ==============================
% 5000*sumsqr(Gs.*S) + 0.01*sumsqr(Gx.*X(1:Nxt,:)) + 50*sumsqr(Gu.*U);  
objective = sumsqr(U) + 10000*sumsqr(S) + 1*(Kt/dt_MPC)*sumsqr(X(1:Nxt,2:end) - Xt_lb(:,1)); %+ 1*sumsqr(X(1:Nxt,:)); %+ sumsqr(X(1:Nxt,:) - X_ref); %+ sumsqr(S);                          
opti.minimize(objective); 

%% ============================================== Dynamics =====================================
% Runge-Kutta 4 integration 
dt_MPC = casadi.MX.sym('dt',1);  
x_MPC = casadi.MX.sym('x',Nxt+ Nxp);
u_MPC = casadi.MX.sym('u',Nxt);
d_MPC = casadi.MX.sym('d',ND);                              % [d_t1, d_t2, d_p]
p_MPC = casadi.MX.sym('p',NP);                    
%
k1_MPC = dynamics_MPC(x_MPC, u_MPC, d_MPC, p_MPC);
k2_MPC = dynamics_MPC(x_MPC + dt_MPC / 2.0 * k1_MPC, u_MPC, d_MPC, p_MPC);
k3_MPC = dynamics_MPC(x_MPC + dt_MPC / 2.0 * k2_MPC, u_MPC, d_MPC, p_MPC);
k4_MPC = dynamics_MPC(x_MPC + dt_MPC * k3_MPC, u_MPC, d_MPC, p_MPC);

intMethod = 1;
if intMethod == 1
    xf = x_MPC + dt_MPC / 6.0 * (k1_MPC + 2 * k2_MPC + 2 * k3_MPC + k4_MPC);
    F_integral = casadi.Function('F_RK4', {x_MPC, u_MPC, d_MPC, p_MPC, dt_MPC}, {xf}, {'x[k]','u[k]','d[k]','p','dt'},{'x[k+1]'});
elseif intMethod == 2
    xf = x_MPC + dt_MPC*dynamics_MPC(x_MPC, u_MPC, d_MPC, p_MPC);
    F_integral = casadi.Function('F_EUL', {x_MPC, u_MPC, d_MPC, p_MPC, dt_MPC}, {xf}, {'x[k]','u[k]','d[k]','p','dt'},{'x[k+1]'});
end

%%
    %% ==================================== Dynamics constraints ===============================
% Initial state boundary condition - including v0, h0 
opti.subject_to(X(:,1)==X0);                                                % initial state condition

% Gap - closing constraint - from t0 to Hp                                              
for k=1:Hp                           
   opti.subject_to(X(:,k+1)==F_integral(X(:,k), U(:,k), D(:,k), P, DT));  
end

%% ==================================== Physical constraints ===============================
for k = 1:Nxt
    opti.subject_to(Xt_lb(k,:)+ S(k,:) <= X(k,:) <= Xt_ub(k,:)+ S(k,:));    % Soft constraint on tank state
    opti.subject_to(S(k,:) >= zeros(1,Hp+1));                               % Slack variable is always positive - Vof >= 0
end

for k = 1:Nxt
    opti.subject_to(U_lb(k,:) <= U(k,:) <= U_ub(k,:));                      % Hard constraint on input                         
end

for k = (Nxt+1):(Nxt+Nxp)
    opti.subject_to(Xp_lb(k-Nxt,:) < X(k,:) <= Xp_ub(k-Nxt,:));             % hard constraint on pipe state
end

%% ====================================== Solver settings ==================================
% Solver options
opts = struct;
opts.ipopt.print_level = 0;                                                     % print enabler to command line
opts.print_time = false;
opts.expand = true;                                                             % makes function evaluations faster
%opts.ipopt.hessian_approximation = 'limited-memory';
opts.ipopt.max_iter = 300;                                                      % max solver iteration
opti.solver('ipopt',opts);                                                      % solver: ipopt(default)

warmStartEnabler = 1;
if warmStartEnabler == 1                                                        % Parametrized Open Loop Control problem with WARM START
    OCP = opti.to_function('OCP',{X0,D,P,X_ref,opti.lam_g,opti.x,DT},{U(:,1),S(:,1),Y(:,1),opti.lam_g,opti.x},{'x0','d','p','x_ref','lam_g','x_init','dt'},{'u_opt','s_opt','y_opt','lam_g','x_init'});
elseif warmStartEnabler == 0                                                    % Parametrized Open Loop Control problem without WARM START 
    OCP = opti.to_function('OCP',{X0,D,P,X_ref,DT},{U(:,1),S(:,1),Y(:,1)},{'x0','d','p','x_ref','dt'},{'u_opt','s_opt','y_opt'});
end

disp('Casadi builder: OK.')

%%
function y = g(z,p3)    
    y = ((z).^(5/3)) ./ (((z) + p3).^(2/3));        
end
