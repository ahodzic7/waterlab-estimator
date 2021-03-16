    F_variance = evalin('base','F_varaiance');
    h1 = [1 zeros(1,5)];
    h2 = [zeros(1,5) 1];
    sigma_x = zeros(2,Hp);
    var_D = magic(2);    % 2x2
    var_model = magic(6); % 6x6
    var_x_prev = magic(6);
    for i = 1:Hp
        var_x = full(F_variance(var_x_prev, var_D, var_model,10));
        sigma_x(1,i) = h1*var_x*h1';
        sigma_x(2,i) = h2*var_x*h2';
        var_x_prev = var_x;
    end