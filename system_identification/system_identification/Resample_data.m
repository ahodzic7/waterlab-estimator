clear all; 
close all;
clear path;
clc; 

%% ================================================ Load Data ================================================
data = dataLoad('fredericia_v2_backflow_6.csv');                                    % Load simulation data 
startDataIndex = 1; 
endDataIndex = size(data,2);
%% ================================================ Prepare Data =============================================
sim_length_s = data(12,size(data,2));
sample_time_in_s = 10;
N_sensors = 4;                                                             % Select section number, i.e. pick number of level sensor data

N_states = N_sensors + 1; % Number of states +1 -> tank 2
N_augmented_states = 0;
N_optimization_variables = N_states;

real_time = data(12,:);
h(1:N_sensors,:) = uConv(data(3:1:3+N_sensors-1,startDataIndex:endDataIndex), [""]);

Q(1,:) = uConv(data(9,startDataIndex:endDataIndex), ["", ""]); % Select in/outflows

Q(2,:) = uConv(data(10,startDataIndex:endDataIndex), ["", ""]);

T2 = uConv(data(8,startDataIndex:endDataIndex), [""]);                       % Select tanks

tank_area = uConv(data(11,1),[""]);

resampled_data = interp1(real_time ,data(2:11,:)',0:sample_time_in_s:sim_length_s)';
new_data = [0:1:sim_length_s/sample_time_in_s;resampled_data;0:sample_time_in_s:sim_length_s];
new_data(11,:) = tank_area;

save('data\resampled_fredericia_v2_backflow_6','new_data')