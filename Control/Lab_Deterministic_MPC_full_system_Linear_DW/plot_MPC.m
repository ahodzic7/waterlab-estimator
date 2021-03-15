    figure(1)
    clf(1)
    elapsed_time = 0:step-1;
    future_time = elapsed_time(end):(elapsed_time(end)+Hp);
    
    subplot(4,1,1)
    title('Tank States');
    hold on
    % Time elapsed in simulation
    plot(elapsed_time, X_sim_num(1,1:step),'b', 'DisplayName','Tank1')
    hold on
    plot(elapsed_time, X_sim_num(6,1:step),'g', 'DisplayName','Tank2')
    hold on
    % Current step
    plot(elapsed_time(end), X_sim_num(1,step),'r*');
    hold on
    plot(elapsed_time(end), X_sim_num(6,step),'r*');
    hold on
    % Future predicitions
    plot(future_time ,X_predict_num(1,:),'b--');
    hold on
    plot(future_time ,X_predict_num(6,:),'g--');
    hold on
    xlim([0,N+Hp])
    hold off
    legend show
    
    subplot(4,1,2)  
    title('Control inputs');
    hold on
    plot(elapsed_time, U_sim_num(1,1:step),'b')
    hold on
    plot(elapsed_time, U_sim_num(2,1:step),'g')
    hold on
    for i = 1:2
    plot(elapsed_time(end), U_sim_num(i,step),'r*');
    hold on
    end
    plot(future_time ,[U_sim_num(1,step),U_out_num(1,:)],'b--');
    hold on
    plot(future_time ,[U_sim_num(2,step),U_out_num(2,:)],'g--');
    hold off
    xlim([0,N+Hp])
    
    subplot(4,1,3)
    title('Slack Variables');
    hold on
    plot(elapsed_time, S_sim_num(1,1:step),'b')
    hold on
    plot(elapsed_time, S_sim_num(2,1:step),'g')
    hold on
    for i = 1:2
    plot(elapsed_time(end), S_sim_num(i,step),'r*');
    hold on
    end
    plot(future_time ,[S_sim_num(1,step),S_out_num(1,:)],'b--');
    hold on
    plot(future_time ,[S_sim_num(2,step),S_out_num(2,:)],'g--');
    hold off
    xlim([0,N+Hp])
    
    subplot(4,1,4)
    title('Disturbance');
    hold on
    plot(disturbance(1,:),'k--')
    hold on
    plot(dist_forcast(1,:),'k')
    hold on
    plot(disturbance(2,:),'r--')
    hold on
    plot(dist_forcast(2,:),'r')
    hold off
    xlim([0,N+Hp])
    
    figure(2)
    clf(2)
    for i=2:5
   % Time elapsed in simulation
    plot(elapsed_time, X_sim_num(i,1:step),'DisplayName',['Pipe ',num2str(i-1)])
    hold on
    % Future predicitions
    plot(future_time ,X_predict_num(i,:),'--');
    hold on
    end
    title('Pipe states');
    hold off
    legend show