clear all;            
clear path;
clc;  
%% ================================================ Prepare data ==============================================
dataLoad;                                                                       % Load simulation data 

endData_sysID = 800;%1500;%size(x,2)-startData;      % 1999
Nx = 4;
Nx_meas = 4;   

h(1:Nx_meas,:) = x(3:end,startData:endData_sysID);
Q(1,:) = u(1,startData:endData_sysID);                           % Select in/outflow
Q(2,:) = y(startData:endData_sysID,1)';
Q(3,:) = d(3,startData:endData_sysID);      

%% ============================================ Idata object ================================================ 
unitConv = 60;
T_original = 0.5;
Ts_data = T_original*t_resample/unitConv;                                                                    % [10min] in simulation/control 

input = [Q(1,:); Q(3,:)]';
output = [h(1:end,:); Q(2,:)]';

data = iddata(output,input,Ts_data);                                            % (y,u,Ts) order
data.TimeUnit = 'minutes';

for i = 1:size(output(:,end),1)
    if output(i,end) <= 0
        output(i,end) = 0;
    end
end

plot(input(:,1) + input(:,2))
hold on
plot(output(:,end))

%% ===================================================== Model ============================================

modelName = 'model_cont';
Ts_model = 0;                                                                   % 0 - continuous model, 1,2,.. - discrete model 
order = [size(output,2) size(input,2) Nx];                                 % [Ny Nu Nx] order


%p = [0.09925   50.9288   10.7652];             % for [dm^3/s] flow units 
p = [0.01925   1000   10];             % for [dm^3/min] flow units 


params = [p, Nx];

initStates = 0.0001*ones(Nx, 1);                                                % assume 0 flow at t0
sys_init = idnlgrey(modelName, order, params, initStates, Ts_model);            % create nlgreyest object

sys_init.TimeUnit = 'seconds';
sys_init.Parameters(1).Name = 'p1';
sys_init.Parameters(2).Name = 'p2';
sys_init.Parameters(3).Name = 'p3';
sys_init.Parameters(4).Name = 'Nx';
sys_init.Parameters(4).Fixed = true;                                            % number of sections fixed

sys_init.SimulationOptions.AbsTol = 1e-10;
sys_init.SimulationOptions.RelTol = 1e-8;
sys_init.SimulationOptions.Solver = 'ode4';                                     % 4th order Runge-Kutte solver - fixed-step size             

sys_init.Parameters(1).Minimum = 0.001;     %sys_init.Parameters(1).Maximum = 0.5;   % parameter constraints
sys_init.Parameters(2).Minimum = 0.001;     %sys_init.Parameters(2).Maximum = 10000;
sys_init.Parameters(3).Minimum = 0.001;     %sys_init.Parameters(3).Maximum = 100;

for i = 1:Nx
sys_init.InitialStates(i).Minimum = 0.000001;                                   % States lower bound constraints
sys_init.InitialStates(i).Maximum = 1;                                          % States upper bound constraints
end

sys_init = setinit(sys_init, 'Fixed', false(Nx,1));

% opt_init = simOptions('InitialCondition',initStates);                           % Simulate model on training data with initial parameters
% y_init = sim(sys_init,data,opt_init);
% 
% EstPlotter;

%% ============================================= Solver options ============================================
opt = nlgreyestOptions;
%Search methods: 'gn' | 'gna' | 'lm' | 'grad' | 'lsqnonlin' | 'auto'
opt.SearchMethod = 'gna'; 
opt.Display = 'on';
opt.SearchOption.MaxIter = 100;
opt.SearchOption.Tolerance = 1e-15; 

%% =============================================== Estimation =============================================
tic 
sys_final = nlgreyest(data,sys_init, opt)                                       % Parameter estimation START

fprintf('\n\nThe search termination condition:\n')
sys_final.Report.Termination

estParams = [sys_final.Parameters(1).Value,...
             sys_final.Parameters(2).Value,...
             sys_final.Parameters(3).Value];

finalStates = sys_final.Report.Parameters.X0;                                   % estimated initial states
toc

%% ========================================== Estimated model ============================================
opt_init = simOptions('InitialCondition',initStates);                           % Simulate model on training data with initial parameters
y_init = sim(sys_init,data,opt_init);
%%
opt_final = simOptions('InitialCondition',finalStates);                         % Simulate model on training data with estimated parameters
y_final = sim(sys_final,data,opt_final);

%% ========================================== Post - process ============================================
EstPlotter;

%%
 P_pipe_min_v2 = estParams;
 save('.\parameters\P_pipe_min_v2','P_pipe_min_v2');

%%
% saveEnabler = 0;
% if saveEnabler == 1
%     switch Nx_meas
%         case 4
%             p_grav_Nx4 = estParams;
%             save('data\p_grav_Nx4','p_grav_Nx4')
%         case 6
%             p_grav_Nx6 = estParams;
%             save('data\p_grav_Nx6','p_grav_Nx6')
%     end 
% end



            
            
            
            