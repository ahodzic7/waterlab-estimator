clear all; 
close all;
clear path;
clc; 

%% ================================================ Load Data ================================================
data = dataLoad('Simulation_data_short.mat');                                    % Load simulation data 
startDataIndex = 1; 
endDataIndex = size(data,2);
%% ================================================ Prepare Data =============================================
N_sensors = 4;                                                             % Select section number, i.e. pick number of level sensor data

N_states = N_sensors + 1;                                                  % Number of states +1 -> tank 2
N_optimization_variables = N_states;
h(1:N_sensors,:) = uConv(data(3:1:3+N_sensors-1,startDataIndex:endDataIndex), ["mmTodm"]);

% Remove the outliers:
h(1,:) = hampel(h(1,:),5);

output = [h(1:1:end,:)'];
Q(1,:) = uConv(data(9,startDataIndex:endDataIndex), ["1/minTo1/s"]);                   % Select in/outflows
Q(2,:) = uConv(data(10,startDataIndex:endDataIndex), ["1/minTo1/s"]); 
input = [Q(1,:)' Q(2,:)'];

T2 = uConv(data(8,startDataIndex:endDataIndex), ["mmTodm"]);                       % Select tanks
output = [output T2'];

if ~isnan(data(7,:))
    Q(3,:) = uConv(data(7,startDataIndex:endDataIndex), []);               % Pump_2 flow
    output = [output Q(3,:)'];
    N_optimization_variables = N_states+1;
end
 

tank_area = uConv(data(11,1),["mm^2Todm^2"]);

%% ============================================ Iddata object ================================================ 
dataTimeStep = 0.5;                                                         % Time step size in seconds

ioData = iddata(output,input,dataTimeStep);                                % (y,u,Ts) (order)

ioData.TimeUnit = 'seconds';

%% ===================================================== Model ============================================
addpath("models"); 
modelName = 'altered_free_flow_model';
Ts_model = 0;                                                              % 0 - continuous model, 1,2,.. - discrete model 
order = [size(output,2) size(input,2) N_states];                                 % [Ny Nu Nx] (order)

if ~isnan(tank_area)
    phi_2 = 1/tank_area;
    % Initial parameters for simulation
%     parametersInitial = [0.005 0.014860 0.00045921 -0.0031593 ...
%     0.008858893835];
%     parametersInitial = [parametersInitial phi_2];

    % Initial parameters for the lab
    parametersInitial = [0.0800 0.1092063882 3.907941840*10^(-8) -0.002793645908 ...
    0.4482285133];
    parametersInitial = [parametersInitial phi_2];
else
    parametersInitial = [0.005 0.014860 0.00045921 -0.0031593 ...
        0.008858893835 1/200];
end
p9 = 0.002;
p10 = 0.002;
systemParamaters = [parametersInitial, N_states, N_optimization_variables, p9, p10];

initStates = 0.0001*ones(N_states, 1);                                     

sys_init = idnlgrey(modelName, order, systemParamaters, initStates, Ts_model);       % create nlgreyest object
sys_init.TimeUnit = 'seconds';
sys_init.Parameters(1).Name = 'theta_1';
sys_init.Parameters(2).Name = 'theta_2';
sys_init.Parameters(3).Name = 'theta_3';
sys_init.Parameters(4).Name = 'theta_4';
sys_init.Parameters(5).Name = 'theta_5';
sys_init.Parameters(6).Name = 'phi_2';
if ~isnan(tank_area)
    sys_init.Parameters(6).Fixed = true;
end
sys_init.Parameters(7).Name = 'Nx';
sys_init.Parameters(7).Fixed = true;                                       % number of sections fixed
sys_init.Parameters(8).Name = 'Nopt_var';
sys_init.Parameters(8).Fixed = true; 
sys_init.Parameters(9).Name = 'theta_9';
sys_init.Parameters(10).Name = 'theta_10';

sys_init = setinit(sys_init, 'Fixed', false(N_states,1));

sys_init.SimulationOptions.AbsTol = 1e-10;
sys_init.SimulationOptions.RelTol = 1e-8;

sys_init.SimulationOptions.Solver = 'ode4';                                % 4th order Runge-Kutte solver - fixed-step size                 


%% ============================================= Solver options ============================================

opt = nlgreyestOptions;
%Search methods: 'gn' | 'gna' | 'lm' | 'grad' | 'lsqnonlin' | 'auto'
opt.SearchMethod = 'gna'; 
opt.Display = 'on';
opt.SearchOption.MaxIter = 200;
opt.SearchOption.Tolerance = 1e-15;

%% =============================================== Estimation =============================================
tic 
sys_final = nlgreyest(ioData,sys_init, opt)                                % Parameter estimation START

fprintf('\n\nThe search termination condition:\n')
sys_final.Report.Termination

estParams = [sys_final.Parameters(1).Value...
             sys_final.Parameters(2).Value...
             sys_final.Parameters(3).Value...
             sys_final.Parameters(4).Value...
             sys_final.Parameters(5).Value...
             sys_final.Parameters(6).Value...
             sys_final.Parameters(9).Value...
             sys_final.Parameters(10).Value];

finalStates = sys_final.Report.Parameters.X0                               % estimated initial states
toc

%% ============================ Simulate model ============================
opt_init = simOptions('InitialCondition',initStates);                      % Simulate model on training data with initial parameters
y_init = sim(sys_init,ioData,opt_init);

opt_final = simOptions('InitialCondition',finalStates);                    % Simulate model on training data with estimated parameters
y_final = sim(sys_final,ioData,opt_final);

%% =========================== Post processing ============================
estParams
data_procesing_plot = 0;
EstPlotter;

%% Save params
save('data\p_grav_Nx4','estParams')

