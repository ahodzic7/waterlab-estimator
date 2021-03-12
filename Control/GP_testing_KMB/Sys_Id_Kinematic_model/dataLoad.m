%% Simulink workspace load
% """
% 1     : tank 1 level
% 2:5   : pipe levels
% 6     : NAN (tank2 inflow) 
% 7     : tank2 level
% 8     : Gravity pipe inflow (input) 
% 9     : tank2 outflow 
% 10    : tank2 area 
% 11    : real time 
% 12    : lateral inflow (disturbance)
% 13    : inflow to tank1 
% 14:17 : [pump1_ref(inflow), pump2_ref(outflow), inflow to tank1_ref, lateral inflow_ref]
% """

addpath('data');

%load('.\data\dataSet_nominal');
%load('.\data\Simulation_data_CCTA_v4');
load('.\data\Simulation_data_WWdata_v2');

labRes = ans;

% Outflow calculation from tank2 level
labRes.Data(1:end-1,6) = (2/10^6)*labRes.Data(1,10)*(labRes.Data(2:end,7) - labRes.Data(1:end-1,7)) + 1*uConv(labRes.Data(1:end-1,9),'mTos');

 
%% ================================================ Prepare data ==============================================                                                                   % ~3.5 [h] long measurement data
startData = 2;                                                                  % the first data point is corrupted
t_resample = 20;                                                                % Resample raw data
endData = size(labRes.Data,1);
% state
x(1,:) = labRes.Data(startData:t_resample:endData-1,1)'/100;                    % [dm]
x(2,:) = labRes.Data(startData:t_resample:endData-1,7)'/100;                    % [dm]
x(3:6,:) = medfilt1(labRes.Data(startData:t_resample:endData-1,2:2+4-1)'/100,3);% [dm]

% input
u(1,:) = uConv(labRes.Data(startData:t_resample:endData-1,8),'none');           % [dm^3/s]
u(2,:) = uConv(labRes.Data(startData:t_resample:endData-1,9),'none');           % [dm^3/s]

% disturbance
d(1,:) = uConv(labRes.Data(startData:t_resample:endData-1,13),'none');          % dt1
d(2,:) = zeros(1,length(d(1,:)));                                               % dt2
d(3,:) = uConv(labRes.Data(startData:t_resample:endData-1,12),'none');          % wp

y_temp = uConv((smooth(labRes.Data(startData:1:endData,6))),'sTom');
y = y_temp(1:t_resample:endData);

% tank constant
conv_mm2Todm2 = 10^-4;
Kt = labRes.Data(1,10)*conv_mm2Todm2;

%plot(y)

%% 
plotEnable = 0;
if plotEnable == 1
figure
subplot(2,1,1)
plot(x(1:2,:)','LineWidth',0.5)
ylabel('Water level','interpreter','latex');
xlabel('Time','interpreter','latex');
title('Tank states','interpreter','latex')

subplot(2,1,2)
plot(x(3:6,:)','LineWidth',0.5)
ylabel('Water level','interpreter','latex');
xlabel('Time','interpreter','latex');
title('Pipe states','interpreter','latex')

figure
plot(u(1:2,:)','LineWidth',0.5)
ylabel('Flow','interpreter','latex');
xlabel('Time','interpreter','latex');
title('Pump flow','interpreter','latex')

figure
plot(d(1:3,:)','LineWidth',0.5)
ylabel('Flow','interpreter','latex');
xlabel('Time','interpreter','latex');
title('Disturbance flow','interpreter','latex')
end

%%

figure
ax(1) = subplot(2,1,1);
plot(uConv(labRes.Data(startData:t_resample:endData,6),'sTom'))
hold on
% plot(uConv(hampel(smooth(labRes.Data(startData:t_resample:endData,6)),2),'sTom'))
% hold on
plot(y)
ax(2) = subplot(2,1,2);
plot(u(1,:))

linkaxes(ax, 'x')


