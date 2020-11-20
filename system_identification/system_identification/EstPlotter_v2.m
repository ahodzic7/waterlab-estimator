
%% Estimated system comparison
plotEnbaler = 1;
if plotEnbaler == 1
    j=0;
    figure
    for i = 1:1:Nx
        ax(i) = subplot(size(output,2),2,i+j);
        plot(0:0.5:size(output(:,1),1)/2-0.5,output(:,i),'b','LineWidth',0.5)
        hold on
        plot(0:0.5:size(output(:,1),1)/2-0.5,y_final.OutputData(:,i),'r','LineWidth',0.5)
        leg = legend('HF Model','Lin. Model','Location','NorthEast');
        set(leg, 'Interpreter', 'latex');
        set(leg,'color','none');
        if i == Nx
            ylabel(['$h_{T2}$' '[$mm$]'],'interpreter','latex');
        else
            ylabel(['$h$' num2str(i) '[$mm$]'],'interpreter','latex');
        end
        if i == 1
            title('Estimated model','interpreter','latex')
        end
        grid on;
        j = j+1;
    end
    xlabel('Time [s]','interpreter','latex');
end
%linkaxes(ax, 'x')

ax(1) = subplot(size(output,2),2,[2 4]);
plot(0:0.5:size(output(:,1),1)/2-0.5,input(:,1),'b','LineWidth',0.5)
hold on
leg = legend('Pipe inflow','Location','NorthEast');
%leg = legend('Tank outflow','Location','NorthEast');
set(leg, 'Interpreter', 'latex');
ylabel(['$Q$' num2str(1) '[$\frac{mm^3}{s}$]'],'interpreter','latex');
title('Flow inputs','interpreter','latex')
grid on;
xlabel('Time [s]','interpreter','latex');

ax(2) = subplot(size(output,2),2,[8 10]);
plot(0:0.5:size(output(:,1),1)/2-0.5,input(:,2),'b','LineWidth',0.5)
hold on
%leg = legend('Pipe inflow','Location','NorthEast');
leg = legend('Tank outflow','Location','NorthEast');
set(leg, 'Interpreter', 'latex');
ylabel(['$Q$' num2str(2) '[$\frac{mm^3}{s}$]'],'interpreter','latex');
grid on;
xlabel('Time [s]','interpreter','latex');
linkaxes(ax, 'x')


if Nx<N_optimization_variables

    Qout_est = y_final.OutputData(:,N_states+1);

    figure
    plot(Q(3,:),'b','LineWidth',0.5)
    hold on
    plot(Qout_est,'r','LineWidth',0.5)
    ylabel('$Q_{out}$  [$\frac{m^3}{s}$]','interpreter','latex');
    xlabel('Time [s]','interpreter','latex');
    leg = legend('Data','Model','Location','NorthEast');
    set(leg, 'Interpreter', 'latex');
    title('Estimated model - flow','interpreter','latex')
end
% 
