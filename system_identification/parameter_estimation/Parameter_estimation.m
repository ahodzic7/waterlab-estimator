clear all; 
close all;
clear path;
clc;

addpath('miscellaneous');
addpath('models');
%% ================================================ Load Data ================================================
while ~exist('dataStructure','var')
    [structureName, filePath] = uigetfile('*.mat','Select data structure');
    fullPath = fullfile(filePath, structureName);

 
    if exist(fullPath, 'file')
        load(fullPath);
        if ~exist('dataStructure','var')
            warningMessage = sprintf('Warning: wrong file:\n%s\n%s', fullPath, 'Choose data structure!');
            uiwait(errordlg(warningMessage));
        end
    else
        warningMessage = sprintf('Warning: mat file does not exist:\n%s', fullPath);
        uiwait(errordlg(warningMessage));
        return;
    end
end

data = dataStructure.data;
for i=1:1:size(data,2)
    if ~isnan(data(3:6,i))
        startDataIndex = i;
        break;
    end
end
endDataIndex = size(data,2);
%% ================================================ Prepare Data =============================================
N_sensors = dataStructure.number_of_sensors;                               % Select section number, i.e. pick number of level sensor data

N_states = N_sensors + 1; % Number of states +1 -> tank 2 // Used to evaluate the Qout form the pipe
N_augmented_states = sum(dataStructure.number_of_augmented_states);
N_optimization_variables = N_states;

% Load the level measurements and convert
h(1:N_sensors,:) = uConv(data(3:1:3+N_sensors-1,startDataIndex:endDataIndex), ...
    [convertCharsToStrings(append(dataStructure.level_units,'To',dataStructure.level_convert_to))]);

% Remove the outliers:
h(1,:) = hampel(h(1,:),8);
h(2,:) = hampel(h(2,:),8);
h(3,:) = hampel(h(3,:),8);
h(4,:) = hampel(h(4,:),8);

output = [h(1:1:end,:)'];

% Select in/outflows
Q(1,:) = uConv(data(9,startDataIndex:endDataIndex), ...
    [convertCharsToStrings(append(dataStructure.flow_units,'To',dataStructure.flow_convert_to))]);

Q(2,:) = uConv(data(10,startDataIndex:endDataIndex), ...
    [convertCharsToStrings(append(dataStructure.flow_units,'To',dataStructure.flow_convert_to))]);
input = [Q(1,:)' Q(2,:)'];

T2 = uConv(data(8,startDataIndex:endDataIndex), ...
    [convertCharsToStrings(append(dataStructure.level_units,'To',dataStructure.level_convert_to))]);                       % Select tanks
output = [output T2'];

if ~isnan(data(7,:))
    Q(3,:) = uConv(data(7,startDataIndex:endDataIndex), ...
        [convertCharsToStrings(append(dataStructure.flow_units,'To',dataStructure.flow_convert_to))]);               % Pump_2 flow
    output = [output Q(3,:)'];
    N_optimization_variables = N_states+1;
end

if dataStructure.lateral_inflow
    Q(4,:) = uConv(data(13,startDataIndex:endDataIndex), ...
        [convertCharsToStrings(append(dataStructure.flow_units,'To',dataStructure.flow_convert_to))]);               % Lateral disturbance flow

    input = [Q(1,:)' Q(2,:)' Q(4,:)'];  
end

tank_area = uConv(data(11,startDataIndex),...
    [convertCharsToStrings(append(dataStructure.level_units, '^2' ,'To',dataStructure.level_convert_to, '^2'))]);

%% ============================================ Iddata object ================================================ 
dataTimeStep = dataStructure.data_time_step;                               % Time step size in seconds

ioData = iddata(output,input,dataTimeStep);                                % (y,u,Ts) (order)

switch dataStructure.data_time_unit
    case 's'
        ioData.TimeUnit = 'seconds';
    case 'min'
        ioData.TimeUnit = 'minutes';
    case 'h'
        ioData.TimeUnit = 'hours';
end
%% ===================================================== Model ============================================
addpath("models"); 
if N_augmented_states > 0
    modelName = append(dataStructure.boundary_condition, '_model_augmented')
else
    modelName = append(dataStructure.boundary_condition, '_model')
end

if dataStructure.lateral_inflow
    modelName = append(modelName, '_lateral_inflow')
end

    
Ts_model = 0;                                                              % 0 - continuous model, 1,2,.. - discrete model 
order = [size(output,2) size(input,2) N_states+N_augmented_states];        % [Ny Nu Nx] (order)

if ~isnan(tank_area)
    phi_2 = 1/tank_area;
    parametersInitial = dataStructure.initial_parameters;
    parametersInitial = [parametersInitial phi_2];
else
    parametersInitial = dataStructure.initial_parameters;
    parametersInitial = [parametersInitial 1/200];
end
systemParamaters = [parametersInitial, N_states, N_optimization_variables, N_augmented_states, dataStructure.number_of_augmented_states];

initialStates = 0.0001*ones(N_states+N_augmented_states, 1);                                     

sys_init = idnlgrey(modelName, order, systemParamaters, initialStates, Ts_model);       % create nlgreyest object
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
sys_init.Parameters(9).Name = 'Naug_states';
sys_init.Parameters(9).Fixed = true;
for i = 1:N_states-1
    sys_init.Parameters(9+i).Fixed = true;
end

if ~isnan(tank_area)
    sys_init = setinit(sys_init, 'Fixed', false(N_states,1));
else
    sys_init = setinit(sys_init, 'Fixed', false(N_states+1,1));
end
sys_init.SimulationOptions.AbsTol = 1e-10;
sys_init.SimulationOptions.RelTol = 1e-8;

sys_init.SimulationOptions.Solver = 'ode1';                                                 

sys_init.Parameters(1).Minimum = 0.03;       % Parameter constraints
sys_init.Parameters(1).Maximum = 1;
sys_init.Parameters(2).Minimum = 0.0001;    
sys_init.Parameters(3).Minimum = 0.0001; 
sys_init.Parameters(5).Minimum = 0.0001;
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

estimatedParameters = [sys_final.Parameters(1).Value...
             sys_final.Parameters(2).Value...
             sys_final.Parameters(3).Value...
             sys_final.Parameters(4).Value...
             sys_final.Parameters(5).Value...
             sys_final.Parameters(6).Value];

estimatedInitialStates = sys_final.Report.Parameters.X0                               % estimated initial states
toc

%% ============================ Simulate model ============================
opt_init = simOptions('InitialCondition',initialStates);                   % Simulate model on training data with initial parameters
y_init = sim(sys_init,ioData,opt_init);

opt_final = simOptions('InitialCondition',estimatedInitialStates);         % Simulate model on training data with estimated parameters
y_final = sim(sys_final,ioData,opt_final);
estimatedParameters

%% =========================== Post processing ============================
EstimatedModelPlotter;
%% Save session
FileName = append('.\results\',erase(structureName,'.mat'),date);
[fPath, fName, fExt] = fileparts(FileName);

if isempty(fExt)  % No '.mat' in FileName
  fExt     = '.mat';
  FileName = fullfile(fPath, [fName, fExt]);
end
if exist(FileName, 'file')
    % Get number of files:
    fDir     = dir(fullfile(fPath, [fName, '*', fExt]));
    fStr     = convertCharsToStrings(sprintf('%s*', fDir.name));
    fNum     = sscanf(fStr, [fName,'(', '%d',')', fExt, '*']);
    if isempty(fNum)
        newNum   = 1;
    else
        newNum   = max(fNum) + 1;
    end
    FileName = fullfile(fPath, [fName, sprintf('(%d)', newNum), fExt]);
end
save(FileName);