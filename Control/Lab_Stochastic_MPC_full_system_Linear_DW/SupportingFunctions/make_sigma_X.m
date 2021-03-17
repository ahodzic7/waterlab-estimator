    F_variance = evalin('base','F_variance');
    h1 = [1 zeros(1,5)];
    h2 = [zeros(1,5) 1];
    sigma_x = zeros(2,Hp);
    
    var_D =  diag([0.0051; 0.0099]);  
    var_model = diag([0.0449 0.0012 0.0018 0.0008 0.0004 0.0449]); 
    var_measuremants = diag([0.005 0.0036 0.0045 0.0028 0.0030 0.005]);
    var_x_prev = var_measuremants;
    
    for i = 1:Hp
        var_x = full(F_variance(var_x_prev, var_D, var_model,10));
        sigma_x(1,i) = h1*var_x*h1';
        sigma_x(2,i) = h2*var_x*h2';
        var_x_prev = var_x;
    end