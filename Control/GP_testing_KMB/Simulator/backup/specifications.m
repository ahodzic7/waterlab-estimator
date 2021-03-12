%% control_specs.m
% """
% Specifies all the control and model specs
% """

addpath('.\control')

%% Basic properties for simulator
Nxt = 2;                            % number of tank states
Nxp = 4;                            % number of channel states 
NP = 3 + 2;                         % number of channel parameters 
ND = 3;                             % number of disturbances

controlType = 2;                    % switch between on/off and MPC

load('parameters\P_pipe_min')       % load Gravity pipe parameters
load('parameters\P_pipe_min_v2')    % load Gravity pipe parameters
load('parameters\Kt')               % load tank parameters
t_resample = 20;                    % Resample raw data - conversion between simulator/MPC time steps
if controlType == 1
    %dataLoad;                      % load data from experiment
end
P_sim = [P_pipe_min_v2, Kt, Kt]';   % all sim parameters

GP = load('.\parameters\GP_parameters.mat');

%% input constraints

if controlType == 1
    input1 = 0;
    input2 = 0;
    u1_on = 9;      u1_off = 0;%4;
    u2_on = 12;     u2_off = 0;%5;
elseif controlType == 2    
    u1_on = 9;      u1_off = 4;
    u2_on = 10.5;   u2_off = 5;
end

%% state constraints 
% Tank constraints
max_t1 = 6.5;     min_t1 = 1.5;             % 6.7 physical maximum 
max_t2 = 6.5;       min_t2 = 1.5;

% Pipe constraints
h_p_max = [0.3;0.3;0.3;0.3];
h_p_min = [0.0001;0.0001;0.0001;0.0001];

%% MPC specs
Hp = 40;
dt_MPC = 0.5*t_resample/60;

