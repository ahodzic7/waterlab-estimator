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

data = load('Pump2_PI.mat').ans;
data = [data.Time, data.Data];
data = [data(:,1),data(:,3),data(:,5),data(:,2),data(:,6)]';
%%
T = data(1,1:60)*0.5;
input = data(3,:)';
input(input==0)=45;
input = input(1:60);
output = data(2,:)';
output = output(1:60);
figure
plot(T, input);
hold on;
plot(T, output);
legend('Pump flow','Pump percentage');
xlabel('Time (s)');
ylabel('flow L/min; pecentage %');

%%
T = data(1,:);
input = data(5,:)';
input(input==100)=100-54;

output = data(4,:)';

figure
plot(T, output);
hold on;
plot(T, input);
legend('Pump flow','Pump percentage');
xlabel('Time (s)');
ylabel('flow L/min; pecentage %');
input = data(5,:)';
output = data(4,:)';
