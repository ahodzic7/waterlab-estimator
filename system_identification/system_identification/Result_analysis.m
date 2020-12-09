%% Residual plots
figure
residuals = [];
for i = 1:1:Nx
    ax(i) = subplot(size(output,2),1,i);
    if i == Nx
        residual = (output(1:end,i)-y_final.OutputData(1:end,i))*100/1.6;
    else
        residual = (output(1:end,i)-y_final.OutputData(1:end,i))*100/1.6;
    end
    plot(0:dataTimeStep:size(output(:,1),1)*dataTimeStep-dataTimeStep,residual,'b','LineWidth',1);
    hold on;
    yline(0,'k','LineWidth',1.5);
    residuals = [residuals residual];
    %hold on
%     plot(y_final.OutputData(:,i),'r','LineWidth',0.5)
%     leg = legend('HF Model','Lin. Model','Location','NorthEast');
%     set(leg, 'Interpreter', 'latex');
%     set(leg,'color','none');
    if i == Nx
        ylabel(['$r_{T2}$' '[$\%$]'],'interpreter','latex');
    else
        ylabel(['$r$' num2str(i) '[$\%$]'],'interpreter','latex');
    end
    if i == 1
        title('Residuals','interpreter','latex')
    end
    grid on;
    j = j+1;
end
% xlabel('Time [s]','interpreter','latex');
% figure
% ax(1) = subplot(size(output,2),1,[2 3 4]);
% histogram(residuals,100,'Normalization','probability');
% hold on
% leg = legend('Pipe inflow','Location','NorthEast');
% leg = legend('Tank outflow','Location','NorthEast');
% set(leg, 'Interpreter', 'latex');
% ylabel(['[$\%$]'],'interpreter','latex');
% title('Distribution of residuals','interpreter','latex')
% grid on;
% xlabel('Residuals [$\%$]','interpreter','latex');

%%
figure
new_residuals = [0 residuals(:,1)'];
% for i = 2:5
%     new_residuals = [new_residuals residuals(:,i)'];
% end
histogram(new_residuals,10,'Normalization','probability');
hold on;
[muHat,sigmaHat] = normfit(residuals(:,1))

x = -0.1:0.01:0.1
y_norm = normpdf(x,muHat(1),sigmaHat(1));
plot(x,y_norm);
% lag = 100;