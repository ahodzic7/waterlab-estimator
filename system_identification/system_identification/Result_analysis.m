%% Residual plots
figure
j = 0;
residuals = [];
for i = 1:1:Nx
    ax(i) = subplot(size(output,2),2,i+j);
    residual = (output(1:end,i)-y_final.OutputData(1:end,i))*100/max(output(:,i));
    plot(0:0.5:size(output(:,1),1)/2-0.5,residual,'b','LineWidth',1);
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
xlabel('Time [min]','interpreter','latex');

ax(1) = subplot(size(output,2),2,[4 6 8]);
histogram(residuals,20,'Normalization','probability');
hold on
%leg = legend('Pipe inflow','Location','NorthEast');
%leg = legend('Tank outflow','Location','NorthEast');
%set(leg, 'Interpreter', 'latex');
%ylabel(['[$\%$]'],'interpreter','latex');
title('Distribution of residuals','interpreter','latex')
grid on;
xlabel('Residuals [$\%$]','interpreter','latex');