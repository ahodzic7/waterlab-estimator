%% control_specs.m
% """
% Specifies all the control and model specs
%
% ControlType : 1 - On / off level control 
%               2 - NMPC with PDE model
%               3 - NMPC with GP model
% """

addpath('.\control')

%% Basic properties for simulator
Nxt = 2;                            % number of tank states
Nxp = 4;                            % number of channel states 
Nx = Nxt + Nxp;                     % all states
Nu = 2;                             % number of inputs
NP = 3 + 2;                         % number of channel parameters 
ND = 3;                             % number of disturbances

load('parameters\P_pipe_min')       % load Gravity pipe parameters
load('parameters\P_pipe_min_v2')    % load Gravity pipe parameters
load('parameters\Kt')               % load tank parameters
t_resample = 20;                    % Resample raw data - conversion between simulator/MPC time steps
if controlType == 1
    dataLoad;                      % load data from experiment
end
P_sim = [P_pipe_min_v2, Kt, Kt]';   % all sim parameters

%% GP dynamics properties

M = 200;                             % dimesnion of K_ZZ covariance matrix used in MPC
Nz = Nx + Nu + ND;                  % dimension of the training set 

GP = load('.\parameters\GP_parameters.mat');
Z_train  = GP.z_train;
Y_train  = GP.y_train;
GP_sigma_F  = GP.sigma_f;
GP_sigma_L  = GP.sigma_L;
GP_sigma    = GP.sigma;

%% input constraints

if controlType == 1
    input1 = 0;
    input2 = 0;
    u1_on = 9;      u1_off = 0;%4;
    u2_on = 12;     u2_off = 0;%5;
elseif controlType == 2 || controlType == 3   
    u1_on = 9;      u1_off = 4;
    u2_on = 12;%10.5;   
    u2_off = 5;
end

%% state constraints 
% Tank constraints
max_t1 = 6.5;     min_t1 = 1.5;             % 6.7 physical maximum 
max_t2 = 6.5;     min_t2 = 1.5;

% Pipe constraints
if controlType == 2
    h_p_max = [0.3;0.3;0.3;0.3];
    h_p_min = [0.0001;0.0001;0.0001;0.0001];
elseif controlType == 3
    h_p_max = 0.3;
    h_p_min = 0.0001;
end

%% MPC specs
Hp = 10;
dt_original = 0.5;
data_timeUnit = 60;
dt_MPC = dt_original*t_resample/data_timeUnit;

