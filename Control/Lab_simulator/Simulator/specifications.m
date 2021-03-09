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

load('parameters\P_pipe_min')     % load Gravity pipe parameters
load('parameters\P_pipe_min_v2')     % load Gravity pipe parameters
dataLoad;                           % load data from experiment

P_sim = [P_pipe_min_v2, Kt, Kt]';      % all sim parameters

%% Properties for ON/OFF controller
% Tank constraints
max_t1 = 6;     min_t1 = 1.5;             % 6.7 physical maximum 
max_t2 = 6;       min_t2 = 2;

% Pipe constraints
h_p_max = [0.3;0.3;0.3;0.3];
h_p_min = [0.0001;0.0001;0.0001;0.0001];

u1_on = 9;      u1_off = 4;
u2_on = 10.5;       u2_off = 4.3;

%% MPC specs
Hp = 40;
dt_MPC = 0.5*t_resample/60;

