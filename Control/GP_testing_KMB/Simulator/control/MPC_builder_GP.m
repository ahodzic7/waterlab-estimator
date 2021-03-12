%% MPC_builder_GP.m
% """
% Build GP-based MPC
% """

addpath('C:\Users\74647\OneDrive - Grundfos\MATLAB\CasAdi') 
import casadi.*
opti = casadi.Opti();                                       

% Nominal dynamics
At = (eye(Nxt)); 
Bt = -diag([dt_MPC/P_sim(4), dt_MPC/P_sim(4)]); 
Et = diag([dt_MPC/P_sim(5), dt_MPC/P_sim(5)]);

A = [At, zeros(Nxt,Nxp); zeros(Nxp,Nx)];
B = [Bt; zeros(Nxp,Nxt)];
E = [Et, zeros(Nxt,1); zeros(Nxp,3)];

Bd = eye(Nx);

%% ============================================ Constraint bounds =============================
U_ub = [u1_on ; u2_on];                                     % Input bounds
U_lb = [u1_off ; u2_off];

Xt_ub = [max_t1 ; max_t2];                                  % Tank state bounds
Xt_lb = [min_t1 ; min_t2];

Xp_ub = h_p_max*ones(Nxp,1);                                % pipe state bounds
Xp_lb = h_p_min*ones(Nxp,1);

H_x = [eye(Nx); -eye(Nx)];                                  % state polytope
b_x = [Xt_ub; Xp_ub; -Xt_lb; -Xp_lb];

H_u = [eye(Nu); -eye(Nu)];                                  % input polytope
b_u = [U_ub; -U_lb];

H_eps = blkdiag(eye(Nx),eye(Nx));                           % slack polytope
%% ============================================ Opti variables ===============================
% single shooting -> X not defined as variable 
U    = opti.variable(Nxt,Hp);                                                                   
EPS  = opti.variable(2*Nx,Hp);                              % state slack

%% ============================================ Opti parameters ==============================
D        = opti.parameter(ND,Hp);                           % disturbance trajectory [d1,d2,d3]

mu_X0    = opti.parameter(Nx,1);                            % initial mu_x  
sigma_X0 = opti.parameter(Nx,Nx);                           % initial sigma

Z_train  = opti.parameter(Nz,M);                            % Z state-input training input
Y_train  = opti.parameter(Nx,M);                            % Y residual training output

GP_sigma_F  = opti.parameter(Nx,1);                         % GP - sigma_f parameter
GP_sigma_L  = opti.parameter(Nx,1);                         % GP - sigma_L parameter
GP_sigma    = opti.parameter(Nx,1);                         % GP - sigma parameter

mu_X_ref = opti.parameter(Nxt,Hp);                          % reference signal

Beta = opti.parameter(Nx,1);                                % GP bias 

%% ============================================ Cell variables ===============================
mu_X       = cell(Hp+1,1); %casadi.MX.sym('mu_X',Nx,Hp+1);
mu_X{1}    = mu_X0;
sigma_X    = cell(Hp+1,1);
sigma_X{1} = sigma_X0;

%% ============================================== GP setup ====================================
% Build GP prior
K_xx = cell(Nx,1);
for a = 1:Nx
    K_xx{a} = (GP_sigma_F(a)^2) * exp(-0.5*(squareform(pdist(GP.z_train(:,1:M)')).^2)/(GP_sigma_L(a)^2));
end

K = cell(1,Nx);
K_xz = cell(1,Nx);                           % K_zx = casadi.MX.sym('kernel',M+1,1);% zeros(M+1,M+1);
K_zz = cell(1,Nx);                         
for k = 1:Hp
    Z = [mu_X{k}; U(:,k); D(:,k)];
    % Build K matrices
    for a = 1:Nx
        temp_K_xz = [];
        for i = 1:M
            temp_K_xz = [temp_K_xz; (GP_sigma_F(a)^2)*exp(-0.5*(1/GP_sigma_L(a)^2)*(Z-Z_train(:,i))'*(Z-Z_train(:,i)))];
        end
        K_xz{a} = temp_K_xz;
        K_zz{a} = GP_sigma_F(a)^2;
    end
    % Build mu_d and sigma_d
    sigma_d = zeros(Nx,1);
    mu_d = zeros(Nx,1);
    temp_mu_d = [];
    temp_sigma_d = [];
    for i = 1:Nx
        temp_mu_d = [temp_mu_d; K_xz{i}'/(K_xx{i} + eye(M)*GP_sigma(i)^2) * Y_train(i,:)'];
        temp_sigma_d = [temp_sigma_d; K_zz{i} - K_xz{i}'/(K_xx{i} + eye(M)*GP_sigma(i)^2) * K_xz{i}];
    end
    mu_d = temp_mu_d;
    sigma_d = temp_sigma_d;
    
    % Mean function gradient 
    grad_mu = zeros(Nx,Nx);
    alpha = cell(Nx,1);
    temp_grad_mu = [];
    for i = 1:Nx
        alpha{i} = (K_xx{i} + eye(M)*GP_sigma(i)^2)\Y_train(i,:)';                        % N x 1
        temp_grad_mu = [temp_grad_mu;((-1/GP_sigma_L(i))*(Z - Z_train)*(K_xz{i}.*alpha{i}))'];
    end
    grad_mu = temp_grad_mu;
    grad_mu = grad_mu(1:Nx,1:Nx);
    
    % Mean and covariance dynamics
    mu_X{k+1} = A*mu_X{k} + B*U(:,k) + E*D(:,k) + Bd*mu_d;%+ Beta;
    sigma_X{k+1} = Bd*diag(sigma_d)*Bd' + (A + Bd*grad_mu)*sigma_X{k}*(A + Bd*grad_mu)';

progressbar(k/Hp) 
end

%% =========================================== Objective function ==============================
objective = 0;
for i = 1:Hp
    objective = objective + sumsqr(mu_X{i+1}(1:Nxt) - mu_X_ref(:,i));
end

objective = 50*(Kt/dt_MPC)*objective + 2*sumsqr(U) + 10*sumsqr(EPS); 
opti.minimize(objective); 

%% ============================================== Constraints ==================================
for k = 2:Hp
    opti.subject_to(H_x*mu_X{k} <= (b_x)); %- norminv(0.5)*abs(H_x)*sqrt( diag(sigma_X{k}) ) + H_eps*EPS(:,k)));   % 
    opti.subject_to(EPS(:,k) >= 0);
end
for k = 1:Hp
    opti.subject_to(H_u*U(:,k) <= b_u);                                               
end

%% Optimization setup 
opts = struct;
opts.ipopt.print_level = 0;                                                    % print enabler to command line
opts.print_time = false;
opts.expand = false;                                                             % makes function evaluations faster
opts.ipopt.max_iter = 100;                                                      % max solver iteration
opti.solver('ipopt',opts);              

%% Setup OCP
tic

% make mu_X cell to MX variable
temp_mu_X = [];
for i = 1:Hp
    temp_mu_X = [temp_mu_X, mu_X{i}];
end

% make sigma_X cell to MX variable
temp_sigma_X = [];
for i = 1:Hp
    temp_sigma_X = [temp_sigma_X, sigma_X{i}];
end

% build control problem
OCP = opti.to_function('OCP',{mu_X0, D, sigma_X0, Z_train, Y_train, GP_sigma_F, GP_sigma_L, GP_sigma, opti.lam_g, opti.x, mu_X_ref, Beta},{U, temp_mu_X, temp_sigma_X, EPS, opti.lam_g, opti.x},{'mu_x0','d','sigma_x0','z_train','y_train','GP_sigma_f','GP_sigma_l','GP_sigma','lam_g','init_x','mu_x_ref','beta'},{'u_opt','mu_x_opt','sigma_x_opt','eps_opt','lam_g','init_x'});

toc

