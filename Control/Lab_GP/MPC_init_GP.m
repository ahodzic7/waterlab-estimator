clear all; clc;

tic
%% Basic properties 
Nxt = 2;                            % number of tank states
Nxp = 4;                            % number of channel states 
Nx = Nxt + Nxp;                     % all states
Nu = 2;                             % number of inputs
Ny = Nx;                            % number of GP outputs
NP = 3 + 2;                         % number of channel parameters 
ND = 3;                             % number of disturbances

load('.\Lab_GP\parameters\Kt')      % load tank parameters

%% GP dynamics properties

M = 30;                             % dimesnion of K_ZZ covariance matrix used in MPC
Nz = Nx + Nu + ND;                  % dimension of the training set 
GP = load('.\Lab_GP\parameters\GP_parameters_lab.mat');     

%% input constraints  
u1_on = 8.3;                           
u1_off = 3.4;                         
u2_on = 16;%17.5;%17           
u2_off = 5.4;                         

%% state constraints 
% Tank constraints
max_t1 = 7;       
min_t1 = 1.8;             % 6.7 physical maximum 
max_t2 = 6.5;     
min_t2 = 1.8;
% Pipe constraints
h_p_max = 0.4;
h_p_min = 0.0001;

%% MPC specs
Hp = 5;                % 40 for Pde mpc
dt_original = 0.5;
data_timeUnit = 60;
t_resample = 20;                    % Resample raw data - conversion between simulator/MPC time steps
dt_MPC = dt_original*t_resample/data_timeUnit;

%% Forecasts 
load('.\Lab_GP\signals\D_sim.mat');
%% reference
load('.\Lab_GP\signals\X_ref_sim.mat');
resample(X_ref_sim,4,1);

%% Replace GP.training_set with full_set
load('.\Lab_GP\signals\y_all.mat');
load('.\Lab_GP\signals\z_all.mat');

GP.y_train = y_all;
GP.z_train = z_all;

%% Build MPC
MPC_builder_GP

%% Initial inputs
lam_g = 0.1;                        % warm start - Lagrange multiplier initializer
x_init = 0.001;  
sigma_X0 = zeros(Nx,Nx);

z_train_subset = GP.z_train(:,1:M); % initial z_subset
y_train_subset = GP.y_train(:,1:M); % initial z_subset

[inv_K_xx_val,K_xx] = K_xx_builder(z_train_subset,GP,Nx,M);                       % initial inv_K_xx

disp('Initialization:')
toc
