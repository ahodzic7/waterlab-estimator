function plot_simulink_mpc(u,S,X0,disturbance,Hp)
    
    clf(123)
    figure(123)
    subplot(2,1,1)
    % Time elapsed in simulation
    % Current step
    plot(0, X0,'r*');
    hold on
    % Future predicitions
    future_time = elapsed_time(end):(elapsed_time(end)+Hp-1 );
    plot(future_time ,X_predict_num,'g');
    xlim([1,N+Hp])
    
    subplot(2,1,2)  
    plot(dist,'k--')
    hold on
    plot(dist_forcast,'k')
    hold on
    
    % Time elapsed in simulation
    elapsed_time = 1:step;
    plot(elapsed_time, U_sim_num(1,1:step),'b')
    plot(elapsed_time, S_sim_num(1,1:step),'g')
    plot(elapsed_time, S_ub_sim_num(1,1:step),'y')
    hold on
    
    % Current step
    plot(step, U_sim_num(1,step),'r*');
    plot(step, S_sim_num(1,step),'r*');
    plot(step, S_ub_sim_num(1,step),'r*');
    hold on
    % Future predicitions
    future_time = elapsed_time(end):(elapsed_time(end)+Hp-1 );
    plot(future_time ,U_out_num,'b--');
    plot(future_time ,S_out_num,'g--');
    plot(future_time ,S_ub_out_num,'y--');
    xlim([1,N+Hp])
    pause(0.25);
end

