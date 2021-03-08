function [dx] = dynamics_MPC(x,u,d,p)
% """ Model dynamics description """
% Continuous dynamics defined in the form: dx/dt = f(x,u,d,p) 
% 
% Parameters:   [p11,p12,p13]
% Disturbances: [d(1),d(2)]
% Inputs:       [u(1),u(2)]
% States:       [ht1,ht2 | hp1,hp2,hp3,hp4]
%
%       h - level
% """

    Nxp = 4; 
    Nxt = 2;
    dx = casadi.MX.zeros(Nxt+ Nxp,1); 

    % Storage element dynamics
    dx(1) = (1/p(4))*(d(1) - u(1));
    dx(2) = (1/p(5))*(d(2) - u(2) + p(2)*g(x(Nxt+ Nxp))); 
    
    % Gravity sewer dyamics - T1 - T2  
    dx(Nxt+ 1) = p(1)*u(1) - p(1)*p(2)*g(x(Nxt+ 1));
    for k = (Nxt+2):(Nxt+Nxp) 
        if k == 4
            dx(k) =  p(1)*p(2)*(g(x(k-1)) - g(x(k))) + p(1)*d(3);  
        else
            dx(k) =  p(1)*p(2)*(g(x(k-1)) - g(x(k)));  
        end
    end

    % Gravity sewer non-linear output
    function y = g(z)    
        y = ((z).^(5/3)) ./ ((z + p(3)).^(2/3));        
    end 
end

