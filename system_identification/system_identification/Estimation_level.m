clear all;            
clear path;
clc;  
%% ================================================ Prepare data ==============================================
dataLoad;                                                                       % Load simulation data 

startData = 1; 
endData = size(g_level,2);

Nx = 4;                                                                         % Select section number, i.e. pick number of level sensor data
if Nx == 6
    h(1:Nx,:) = g_level(1:8:end-1,startData:endData);
elseif Nx == 4
    h(1:Nx,:) = g_level(6:12:end,startData:endData);
elseif Nx == 8
    h(1:Nx,:) = g_level(1:6:end,startData:endData);
end

Q(1,:) = uConv(g_flow(1,startData:endData),'sTo10m');                           % Select in/outflow
Q(2,:) = uConv(g_flow(end,startData:endData),'sTo10m');

%% ============================================ Idata object ================================================ 
Ts_data = 1;                                                                    % [10min] in simulation/control 

input = Q(1,:)';
output = [h(1:1:end-1,:); Q(2,:)]';

data = iddata(output,input,Ts_data);                                            % (y,u,Ts) (order)

%% ===================================================== Model ============================================

modelName = 'model_cont';
Ts_model = 0;                                                                   % 0 - continuous model, 1,2,.. - discrete model 
order = [size(output,2) size(input,2) Nx];                                      % [Ny Nu Nx] (order)

switch(Nx)                                                                      % select initial parameters
    case 8
      p = [0.03, 50, 0.2]; 
    case 6
      p = [0.01, 40, 0.2];  
    case 4
      p = [0.01, 1.5, 0.2]; 
end

params = [p, Nx];

initStates = 0.0001*ones(Nx, 1);                                                % assume no flow at t0
sys_init = idnlgrey(modelName, order, params, initStates, Ts_model);            % create nlgreyest object

sys_init.TimeUnit = 'minutes';
sys_init.Parameters(1).Name = 'p1';
sys_init.Parameters(2).Name = 'p2';
sys_init.Parameters(3).Name = 'p3';
sys_init.Parameters(4).Name = 'Nx';
sys_init.Parameters(4).Fixed = true;                                            % number of sections fixed
size(sys_init);

sys_init.SimulationOptions.AbsTol = 1e-10;
sys_init.SimulationOptions.RelTol = 1e-8;

sys_init.SimulationOptions.Solver = 'ode4';                                         % 4th order Runge-Kutte solver - fixed-step size                 

sys_init.Parameters(1).Minimum = 0.001;     sys_init.Parameters(1).Maximum = 0.5;   % Parameter constraints
sys_init.Parameters(2).Minimum = 0.001;     sys_init.Parameters(2).Maximum = 10000;
sys_init.Parameters(3).Minimum = 0.001;     sys_init.Parameters(3).Maximum = 0.6;

for i = 1:Nx
sys_init.InitialStates(i).Minimum = 0.000001;                                       % States lower bound constraints
end

%% ============================================= Solver options ============================================

opt = nlgreyestOptions;
%Search methods: 'gn' | 'gna' | 'lm' | 'grad' | 'lsqnonlin' | 'auto'
opt.SearchMethod = 'gna'; 
opt.Display = 'on';
opt.SearchOption.MaxIter = 50;
opt.SearchOption.Tolerance = 1e-15;

%% =============================================== Estimation =============================================
tic 
sys_final = nlgreyest(data,sys_init, opt)                                           % Parameter estimation START

fprintf('\n\nThe search termination condition:\n')
sys_final.Report.Termination

estParams = [sys_final.Parameters(1).Value...
             sys_final.Parameters(2).Value,...
             sys_final.Parameters(3).Value];

finalStates = sys_final.Report.Parameters.X0;                                       % estimated initial states
toc

%% ========================================== Simulate model =============================================
opt_init = simOptions('InitialCondition',initStates);                               % Simulate model on training data with initial parameters
y_init = sim(sys_init,data,opt_init);

opt_final = simOptions('InitialCondition',finalStates);                             % Simulate model on training data with estimated parameters
y_final = sim(sys_final,data,opt_final);

%% ========================================== Post - process ============================================
estParams
EstPlotter;

%% Save params
saveEnabler = 0;
if saveEnabler == 1
    switch Nx
        case 4
            p_grav_Nx4_T2_T3 = estParams;
            save('data\p_grav_Nx4_T2_T3','p_grav_Nx4_T2_T3')
        case 6
            p_grav_Nx6_T2_T3 = estParams;
            save('data\p_grav_Nx6_T2_T3','p_grav_Nx6_T2_T3')
    end 
end
