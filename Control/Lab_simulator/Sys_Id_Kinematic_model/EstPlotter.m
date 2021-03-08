
%% Initial system comparison

plotEnbaler = 1;
if plotEnbaler == 1
figure
for i = 1:1:Nx_meas
ax(i) = subplot(size(output,2)-1,1,i);
plot(output(:,i),'b','LineWidth',0.5)
hold on
plot(y_init.OutputData(:,i),'r','LineWidth',0.5)
ylabel(['$h$' num2str(i) '[$m$]'],'interpreter','latex');
leg = legend('Data','Model','Location','NorthEast');
set(leg, 'Interpreter', 'latex');
end
linkaxes(ax, 'x')

% Initial outlet flow comparison
% Calculate outlet flow with initial parameters
%Qout_init = params(2) *  g(y_init.OutputData(:,Nx_meas),params(3));
Qout_init = y_init.OutputData(:,Nx_meas+1);

figure
plot(output(:,end),'b','LineWidth',0.5)
hold on
plot(Qout_init,'r','LineWidth',0.5)
ylabel('$Q_{out}$  [$\frac{m^3}{h}$]','interpreter','latex');
xlabel('Time [10 min]','interpreter','latex');
leg = legend('Data','Model','Location','NorthEast');
set(leg, 'Interpreter', 'latex');
title('Outflow calculation with initial parameters','interpreter','latex')
%ylim([0,0.2])
end

%% Estimated system comparison
plotEnbaler = 1;
if plotEnbaler == 1
figure
for i = 1:1:Nx_meas
ax(i) = subplot(size(output,2),1,i);
plot(output(:,i),'b','LineWidth',0.5)
hold on
plot(y_final.OutputData(:,i),'r','LineWidth',0.5)
leg = legend('Data','Model','Location','NorthEast');
set(leg, 'Interpreter', 'latex');
ylabel(['$h$' num2str(i) '[$dm$]'],'interpreter','latex');
%ylim([0, max(y_final.OutputData(:,1)) + 0.03])
xlim([0, length(output(:,1))])
if i == 1
    title('Water level estimation','interpreter','latex')
end
end
%xlabel('Time [s]','interpreter','latex');
end


% Estimated outlet flow comparison
% Calculate outlet flow with estimated parameters
%Qout_est = estParams(2) *  g(y_final.OutputData(:,Nx),estParams(3));
Qout_est = y_final.OutputData(:,Nx_meas+1);

ax(Nx_meas+1) = subplot(size(output,2),1,Nx_meas+1);
plot(smooth(output(:,end)),'b','LineWidth',0.5)
hold on
plot(Qout_est,'r','LineWidth',1)
ylabel('$Flow$  [$\frac{l}{s}$]','interpreter','latex');
xlabel('Time [s]','interpreter','latex');
leg = legend('Data','Model','Location','NorthEast');
set(leg, 'Interpreter', 'latex');
title('Discharge flow estimation','interpreter','latex')
%ylim([0,0.3])
xlim([0, length(output(:,1))])

linkaxes(ax, 'x')

%% g(z) non-linear function 
function y = g(z,p3)    
    y = ((z).^(5/3)) ./ ((z + p3).^(2/3));        
end
