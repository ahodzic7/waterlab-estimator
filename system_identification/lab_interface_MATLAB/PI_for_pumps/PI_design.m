clear all;
close all;
clc;

%% Data Load

% Loading the data into a .mat file
% 
% The data is in the following form:
% 
% [...
% 1-Time; ... 
% 2-tank1_depth; ...
% 3-pipe1_height; 4-pipe2_height; 5-pipe3_height; 6-pipe4_depth; ...
% 7-tank2_inflow; 8-tank2_depth; 
% 9-pump1_flow; 10-pump2_flow; ...
% 11-tank2_area ...
% ]

data = load('Pump1_PI.mat').ans;
data = [data.Time, data.Data];
data = [data(:,1),data(:,3),data(:,5),data(:,2),data(:,6)]';
%%
figure
plot(data(1,:), data(2,:));
hold on;
plot(data(1,:), data(3,:));
legend('Pump flow','Pump percentage');
xlabel('Time (s)');
ylabel('flow L/min; pecentage %');
%%
figure
plot(data(1,:), data(4,:));
hold on;
plot(data(1,:), data(5,:));
legend('Pump flow','Pump percentage');
xlabel('Time (s)');
ylabel('flow L/min; pecentage %');
