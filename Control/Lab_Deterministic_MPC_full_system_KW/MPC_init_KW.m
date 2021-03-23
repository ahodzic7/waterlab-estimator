clear all;
clc

%% ============================================ Control setup ======================================
addpath('.\Lab_simulator\Simulator\lab signal design\generated_data');
load('.\Lab_simulator\Simulator\lab signal design\generated_data\d_lab');
addpath('.\Lab_simulator\Simulator')

dataLoad; 

%% Basic properties for simulator
Nxt = 2;                            % number of tank states
Nxp = 4;                            % number of channel states 
NP = 3 + 2;                         % number of channel parameters 
ND = 3;                             % number of disturbances

load('.\Lab_simulator\Simulator\parameters\P_pipe_min')     % load Gravity pipe parameters
load('.\Lab_simulator\Simulator\parameters\P_pipe_min_v2')     % load Gravity pipe parameters

P_sim = [P_pipe_min_v2, Kt, Kt]';      % all sim parameters

%% Properties for ON/OFF controller
% Tank constraints
max_t1 = 7.02;     min_t1 = 1.5;             % 6.7 physical maximum 
max_t2 = 7.02;     min_t2 = 1.5;

% Pipe constraints
h_p_max = [0.3;0.3;0.3;0.3];
h_p_min = [0.0001;0.0001;0.0001;0.0001];

u1_on = 8.3;          
u1_off = 3.4;
u2_on = 16;%19.5;         
u2_off = 5.4;

%% MPC specs
Hp = 40;
dt_MPC = 0.5*t_resample/60;

%% Forecasts 
load('Lab_Deterministic_MPC_full_system_KW\D_sim.mat');
%% reference
load('Lab_Deterministic_MPC_full_system_KW\X_ref_sim.mat');

MPC_builder;


