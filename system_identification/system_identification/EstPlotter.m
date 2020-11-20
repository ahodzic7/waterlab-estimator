%% Initial system comparison
if data_procesing_plot
    Nx = N_sensors + 1;
    figure
    for i = 1:1:Nx
        ax(i) = subplot(size(output,2),1,i);
        plot(output(:,i),'b','LineWidth',0.5)
        hold on
        if i == Nx
            ylabel(['$h_{T2}$'  '[$m$]'],'interpreter','latex');  
        else
            ylabel(['$h$' num2str(i) '[$m$]'],'interpreter','latex');
        end
        if i == 1
            title('Data - pipe heights and tank 2 height','interpreter','latex')
        end
    end
    xlabel('Time [s]','interpreter','latex');
linkaxes(ax, 'x')
else
plotEnbaler = 1;
if plotEnbaler == 1
    Nx = N_sensors + 1;
    figure
    for i = 1:1:Nx
        ax(i) = subplot(size(output,2),1,i);
        plot(output(:,i),'b','LineWidth',0.5)
        hold on
        plot(y_init.OutputData(:,i),'r','LineWidth',0.5)
        ylabel(['$h$' num2str(i) '[$m$]'],'interpreter','latex');
        leg = legend('Data','Model','Location','NorthEast');
        set(leg, 'Interpreter', 'latex');
        if i == 1
            title('Initial model - level','interpreter','latex')
        end
    end
    xlabel('Time [s]','interpreter','latex');
linkaxes(ax, 'x')

% Qout_init = y_init.OutputData(:,Nx+1);
% 
% figure
% plot(Q(2,:),'b','LineWidth',0.5)
% hold on
% plot(Qout_init,'r','LineWidth',0.5)
% ylabel('$Q_{out}$  [$\frac{m^3}{h}$]','interpreter','latex');
% xlabel('Time [10 min]','interpreter','latex');
% leg = legend('Data','Model','Location','NorthEast');
% set(leg, 'Interpreter', 'latex');
% title('Initial model - flow','interpreter','latex')
end

%% Estimated system comparison
plotEnbaler = 1;
if plotEnbaler == 1
    figure
    for i = 1:1:Nx
        ax(i) = subplot(size(output,2),1,i);
        plot(output(:,i),'b','LineWidth',0.5)
        hold on
        plot(y_final.OutputData(:,i),'r','LineWidth',0.5)
        leg = legend('HF Model','Lin. Model','Location','NorthEast');
        set(leg, 'Interpreter', 'latex');
        set(leg,'color','none');
        if i == Nx
            ylabel(['$h_{T2}$' '[$m$]'],'interpreter','latex');
        else
            ylabel(['$h$' num2str(i) '[$m$]'],'interpreter','latex');
        end
        if i == 1
            title('Estimated model','interpreter','latex')
        end
        grid on;
    end
    xlabel('Time [min]','interpreter','latex');
end
linkaxes(ax, 'x')

figure
    for i = 1:1:2
        ax(i) = subplot(size(input,2),1,i);
        plot(input(:,i),'b','LineWidth',0.5)
        hold on
        if i == 1
            leg = legend('Pipe inflow','Location','NorthEast');
        else
            leg = legend('Tank outflow','Location','NorthEast');
        end
        set(leg, 'Interpreter', 'latex');
        ylabel(['$Q$' num2str(i) '[$\frac{m^3}{s}$]'],'interpreter','latex');
        if i == 1
            title('Flow inputs','interpreter','latex')
        end
        grid on;
    end
    xlabel('Time [min]','interpreter','latex');
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
end
% 
