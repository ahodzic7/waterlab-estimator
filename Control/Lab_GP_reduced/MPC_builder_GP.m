%% MPC_builder_GP.m

addpath('C:\Users\Casper and Adis\Desktop\casadi-windows-matlabR2016a-v3.5.5') 
%addpath('C:\Users\74647\OneDrive - Grundfos\MATLAB\CasAdi') 
import casadi.*
opti = casadi.Opti();                                       

% Nominal dynamics
At = (eye(Nxt)); 
Bt = -diag([dt_MPC/Kt, dt_MPC/Kt]); 
Et = diag([dt_MPC/Kt, dt_MPC/Kt]);

A = [At, zeros(Nxt,Nxp); zeros(Nxp,Nxt), zeros(Nxp,Nxp)];
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

%H_eps = blkdiag(eye(Nx),eye(Nx));                           % polytope with only upper bound slack 
%% ============================================ Opti variables ===============================
dU    = opti.variable(Nxt,Hp);                                                                   
%EPS  = opti.variable(2*Nx,Hp);                              % state slack

%% ============================================ Opti parameters ==============================
D = opti.parameter(ND,Hp);                                  % disturbance trajectory [d1,d2,d3]

mu_X0 = opti.parameter(Nx,1);                               % initial mu_x  
u0 = opti.parameter(Nxt,1);                                 % previous input
sigma_X0 = opti.parameter(Nx,Nx);                           % initial sigma

Z_train  = opti.parameter(Nz,M);                            % Z state-input training input
Y_train  = opti.parameter(Nx,M);                            % Y residual training output
GP_sigma_F  = opti.parameter(Nx,1);                         % GP - sigma_f hyper parameter
GP_sigma_L  = opti.parameter(Nx,1);                         % GP - sigma_L hyper parameter

inv_K_xx = opti.parameter(M,M*Nx);                          % Covariance matrices for each 'a' output dimension

mu_X_ref = opti.parameter(Nxt,Hp);                          % reference signal

%% ============================================ Casadi MX variables ===============================
mu_X       = casadi.MX(Nx,Hp+1);                                         
mu_X(:,1)  = mu_X0;                                         % init. mean (measured state)
sigma_X    = casadi.MX(Nx, (Hp+1)*Nx);
sigma_X(:,1:1*Nx) = sigma_X0;                               % init variance (zero)

%% ============================================== GP setup ====================================
% Build GP prior: K_xx - Gram matrix of data points 
%                    z - testing point                         
K_xz = casadi.MX(M,Nx);                           
K_zz = casadi.MX(1,Nx);
U = cumsum(dU,2) + u0;                                        % integral action 
for k = 1:Hp
    Z = [mu_X(:,k); U(:,k); D(:,k)];
    % Build K_xz and K_zz matrices
    for a = 1:Nx
        temp_K_xz = casadi.MX(M,1);
        Z_C = GP.C{a}*(Z - Z_train);                          % pick testing, training dimensions
        for i = 1:M
            temp_K_xz(i) =(GP_sigma_F(a)^2)*exp(-0.5*(1/GP_sigma_L(a).^2)*Z_C(:,i)'*Z_C(:,i));
        end
        K_xz(:,a) = temp_K_xz;
        K_zz(:,a) = GP_sigma_F(a)^2;
    end
    % Build mu_d and sigma_d (GP)
    mu_d = casadi.MX(Nx,1);
    sigma_d = casadi.MX(Nx,1);
    for i = 1:Nx
        mu_d(i) = K_xz(:,i)' * inv_K_xx(:,(i-1)*M+1:i*M) * Y_train(i,:)';
        sigma_d(i) = K_zz(:,i) - K_xz(:,i)' * inv_K_xx(:,(i-1)*M+1:i*M) * K_xz(:,i);
    end
    % Mean gradient 
    grad_mu = casadi.MX(Nz,Nx);
    alpha = opti.parameter(M,Nx);
    for a = 1:Nx
        alpha(:,a) = inv_K_xx(:,(a-1)*M+1:a*M) * Y_train(a,:)';  
        grad_mu(:,a) = ((-1/GP_sigma_L(a).^2)*(GP.C{a}'*GP.C{a})*(Z - Z_train)*(K_xz(:,a).*alpha(:,a)))';
    end
    grad_mu = grad_mu(1:Nx,1:Nx);
    % Mean and covariance dynamics - uncertainty propagation
    mu_X(:,k+1) = A*mu_X(:,k) + B*U(:,k) + E*D(:,k) + Bd*mu_d;%+ Beta;
    sigma_X(:,((k-1)*Nx+1)+Nx:(k*Nx)+Nx) = Bd*diag(sigma_d)*Bd' + (A + Bd*grad_mu)*sigma_X(:,((k-1)*Nx+1):(k*Nx))*(A + Bd*grad_mu)';
end

%% =========================================== Objective function ==============================
objective_ref_t1 = sumsqr(mu_X(1,2:end) - mu_X_ref(1,:));
objective_ref_t2 = sumsqr(mu_X(2,2:end) - mu_X_ref(2,:));

objective_sigma = 0;
for i = 1:Hp
    objective_sigma = objective_sigma + trace(sigma_X(1:Nxt,((i-1)*Nx+1):(i*Nx)-Nxp));
end

objective_all = 2*(Kt/dt_MPC)*(objective_ref_t1 + objective_ref_t2 + objective_sigma ) + 0.5*sumsqr(dU); %+ 100*(Kt/dt_MPC)*sumsqr(EPS);    %5000 on EPS
opti.minimize(objective_all); 

%% ============================================== Constraints ==================================
for k = 2:Hp
    opti.subject_to(H_x*mu_X(:,k) <= ((b_x))); %+ H_eps*EPS(:,k)));
    %opti.subject_to(EPS(:,k) >= 0);
end
for k = 1:Hp
    opti.subject_to(H_u*U(:,k) <= b_u);                                               
end

%% Optimization setup 
opts = struct;
% opts.ipopt.print_level = 0;                                                     % print enabler to command line
% opts.print_time = false;
opts.expand = true;                                                            % makes function evaluations faster
opts.ipopt.max_iter = 150;                                                      % max solver iteration
opti.solver('ipopt',opts); 

%% Setup OCP
warmstartEnabler = 1;
if warmstartEnabler == 1
OCP = opti.to_function('OCP',{mu_X0, D, sigma_X0, Z_train, Y_train, GP_sigma_F, GP_sigma_L, opti.lam_g, opti.x, mu_X_ref, inv_K_xx, u0},...
    {U, mu_X(:,1:Hp), sigma_X(:,1:Nx*Hp), opti.lam_g, opti.x},...
    {'mu_x0','d','sigma_x0','z_train','y_train','GP_sigma_f','GP_sigma_l','lam_g','init_x','mu_x_ref','inv_K_xx','u0'},...
    {'u_opt','mu_x_opt','sigma_x_opt','lam_g','init_x'});
elseif warmstartEnabler == 0
OCP = opti.to_function('OCP',{mu_X0, D, sigma_X0, Z_train, Y_train, GP_sigma_F, GP_sigma_L, mu_X_ref, inv_K_xx,u0},...
    {U, mu_X(:,1:Hp), sigma_X(:,1:Nx*Hp)},...
    {'mu_x0','d','sigma_x0','z_train','y_train','GP_sigma_f','GP_sigma_l','mu_x_ref','inv_K_xx','u0'},...
    {'u_opt','mu_x_opt','sigma_x_opt'});
end
