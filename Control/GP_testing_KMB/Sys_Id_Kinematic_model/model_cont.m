function [dx, y] = model_cont(t, x, u, p1, p2, p3, Nx, varargin)
% Discrete time nlgreyest model for the kinematic wave level-based model. 

dx = zeros(Nx,1);
y = zeros(5,1);
%% State equation
dx(1) =  p1 * u(1) - p1*p2 * g(x(1)); 

for i = 2:Nx 
    if i == 2
        dx(i) =  p1*p2*(g(x(i-1)) - g(x(i))) + p1*u(2);
    else
        dx(i) =  p1*p2*(g(x(i-1)) - g(x(i)));  
    end
end

%% Output equations

% 
%     y(1) = x(3);
%     y(2) = x(4);
%     y(3) = x(6);
%     y(4) = x(8);
%     y(5) = p2*g(x(Nx));  

    y(1) = x(1);
    y(2) = x(2);
    y(3) = x(3);
    y(4) = x(4);
    y(5) = p2*g(x(Nx));   
    

%% g(z) non-linear function 
function y = g(z)    
    y = ((z).^(5/3)) ./ ((z + p3).^(2/3));        
end
end