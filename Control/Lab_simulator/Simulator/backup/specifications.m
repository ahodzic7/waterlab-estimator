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

load('.\parameters\P_pipe_min')     % load Gravity pipe parameters
load('.\parameters\P_pipe_min_v2')     % load Gravity pipe parameters
dataLoad;                           % load data from experiment

P_sim = [P_pipe_min_v2, Kt, Kt]';      % all sim parameters

%% Properties for ON/OFF controller
% Tank constraints
max_t1 = 5.6;     min_t1 = 2.8;
max_t2 = 4;       min_t2 = 2.1;

% Pipe constraints
h_p_max = [0.3;0.3;0.3;0.3];
h_p_min = [0;0;0;0];

u1_on = 7.5;      u1_off = 0;
u2_on = 10;       u2_off = 0;

input1 = u1_off;
input2 = u2_on;


%% MPC specs
Hp = 20;

% u1_on = 7.5;      u1_off = 4.5;
% u2_on = 10;       u2_off = 4;

