%% for the lab

figure
for i = 1:1:Nx-1
    ax(i) = subplot(size(output,2)-1,1,i);
    plot(500:dataTimeStep:1000-dataTimeStep,output(1000:2000-1,i),'b','LineWidth',2)
    hold on
    plot(500:dataTimeStep:1000-dataTimeStep,four_states.OutputData(1000:2000-1,i),'r-','LineWidth',1)
    hold on
    plot(500:dataTimeStep:1000-dataTimeStep,eight_states.OutputData(1000:2000-1,i),'g','LineWidth',1)
    hold on;
    plot(500:dataTimeStep:1000-dataTimeStep,twenty_states.OutputData(1000:2000-1,i),'k','LineWidth',1)
    leg = legend('Data','Nx = 4','Nx = 8', 'Nx = 20','Location','NorthEast');
    set(leg, 'Interpreter', 'latex');
    %set(leg,'color','none');
    ylabel(['$h$' num2str(i) '[$dm$]'],'interpreter','latex');

    grid on;
end
xlabel('Time [s]','interpreter','latex');
%% for Fredericia
figure
for i = 1:1:Nx-1
    ax(i) = subplot(size(output,2)-1,1,i);
    plot(50:dataTimeStep:2000-dataTimeStep,output(5:200-1,i),'b','LineWidth',2)
    hold on
    plot(50:dataTimeStep:2000-dataTimeStep,four_states.OutputData(5:200-1,i),'r-','LineWidth',1)
    hold on
    plot(50:dataTimeStep:2000-dataTimeStep,eight_states.OutputData(5:200-1,i),'g','LineWidth',1)
    hold on;
    plot(50:dataTimeStep:2000-dataTimeStep,twenty_states.OutputData(5:200-1,i),'k','LineWidth',1)
    leg = legend('Data','Nx = 4','Nx = 8', 'Nx = 20','Location','NorthEast');
    set(leg, 'Interpreter', 'latex');
    %set(leg,'color','none');
    ylabel(['$h$' num2str(i) '[$m$]'],'interpreter','latex');
    grid on;
end
xlabel('Time [s]','interpreter','latex');
%%
output_combined = [output(:,1)' output(:,2)' output(:,3)' output(:,4)'];
four_states_combined = [four_states.OutputData(:,1)' four_states.OutputData(:,2)' four_states.OutputData(:,3)' four_states.OutputData(:,4)'];
eight_states_combined = [eight_states.OutputData(:,1)' eight_states.OutputData(:,2)' eight_states.OutputData(:,3)' eight_states.OutputData(:,4)'];
twelve_states_combined = [twelve_states.OutputData(:,1)' twelve_states.OutputData(:,2)' twelve_states.OutputData(:,3)' twelve_states.OutputData(:,4)'];
sixteen_states_combined = [sixteen_states.OutputData(:,1)' sixteen_states.OutputData(:,2)' sixteen_states.OutputData(:,3)' sixteen_states.OutputData(:,4)'];
twenty_states_combined = [twenty_states.OutputData(:,1)' twenty_states.OutputData(:,2)' twenty_states.OutputData(:,3)' twenty_states.OutputData(:,4)'];

RMSE_4 = sqrt(sum((four_states_combined-output_combined).^2)/size(output_combined,2))/1.6;
RMSE_8 = sqrt(sum((eight_states_combined-output_combined).^2)/size(output_combined,2))/1.6;
RMSE_12 = sqrt(sum((twelve_states_combined-output_combined).^2)/size(output_combined,2))/1.6;
RMSE_16 = sqrt(sum((sixteen_states_combined-output_combined).^2)/size(output_combined,2))/1.6;
RMSE_20 = sqrt(sum((twenty_states_combined-output_combined).^2)/size(output_combined,2))/1.6;
figure
bar([4,8,12,16,20], [RMSE_4, RMSE_8, RMSE_12, RMSE_16, RMSE_20]);
xlabel('Number of states Nx')
ylabel('RMSE')