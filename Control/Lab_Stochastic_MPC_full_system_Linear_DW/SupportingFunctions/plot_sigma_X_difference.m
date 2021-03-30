    F_variance = evalin('base','F_variance');
    F_variance_ol = evalin('base','F_variance_ol');
    
    h1 = [1 zeros(1,5)];
    h2 = [zeros(1,5) 1];
    sigma_x = zeros(2,Hp);
    
    var_D =  diag([0.0051; 0.0099]);  
    var_model = diag([0.0449 0.0012 0.0018 0.0008 0.0004 0.0449]); 
    var_measuremants = diag([0.005 0.0036 0.0045 0.0028 0.0030 0.005]);
    var_x_prev = var_measuremants;
    var_x_prev_ol = var_measuremants;
    for i = 1:50
        var_x = full(F_variance(var_x_prev, var_D, var_model,K,10));
        var_x_ol = full(F_variance_ol(var_x_prev_ol, var_D, var_model,10));
        sigma_x(1,i) = h1*var_x*h1';
        sigma_x(2,i) = h2*var_x*h2';
        sigma_x_ol(1,i) = h1*var_x_ol*h1';
        sigma_x_ol(2,i) = h2*var_x_ol*h2';
        var_x_prev = var_x;
        var_x_prev_ol = var_x_ol;
    end
    
    analyse = 1;
    if analyse
        figure
        plot(sigma_x(1,:),'r');
        hold on;
        plot(sigma_x_ol(1,:),'r--');
        
        plot(sigma_x(2,:),'b');
        hold on;
        plot(sigma_x_ol(2,:),'b--');
        legend('CL-dynamics T1','OL-dynamics T1','CL-dynamics T2','OL-dynamics T2','Location','NorthWest')
        title('Variance Dynamics');
        xlabel('Time k [10 sec]');
        ylabel('Variance [dm]');
    end
    