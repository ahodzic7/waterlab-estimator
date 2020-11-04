clear all; 
close all;
clear path;
clc; 

%% ================================================ Load Data ================================================
data = dataLoad('Simulation_data_long_2.mat');                                    % Load simulation data 
startDataIndex = 24; 
endDataIndex = size(data,2);
%% ================================================ Prepare Data =============================================
N_sensors = 4;                                                             % Select section number, i.e. pick number of level sensor data

N_states = N_sensors + 1;                                                  % Number of states +1 -> tank 2
N_optimization_variables = N_states;
h(1:N_sensors,:) = uConv(data(3:1:6,startDataIndex:endDataIndex), []);
figure
plot(h(1,:));
hold on;
h(1,:) = hampel(h(1,:),5);
plot(h(1,:));
output = [h(1:1:end,:)'];

Q(1,:) = uConv(data(9,startDataIndex:endDataIndex), [""]);                   % Select in/outflows
Q(2,:) = uConv(data(10,startDataIndex:endDataIndex), [""]); 
input = [Q(1,:)' Q(2,:)'];

T2 = uConv(data(8,startDataIndex:endDataIndex), []);                       % Select tanks
output = [output T2'];

if ~isnan(data(7,:))
    Q(3,:) = uConv(data(7,startDataIndex:endDataIndex), []);               % Pump_2 flow
    output = [output Q(3,:)'];
    N_optimization_variables = N_states+1;
end
 

tank_area = data(11,1);

data_procesing_plot = 1;
EstPlotter;