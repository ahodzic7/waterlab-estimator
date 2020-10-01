function [dx, y] = model_cont(t, x, u, p1, p2, p3, Nx, varargin)
% Continuous time nlgreyest model for the kinematic wave level-based model. 

dx = zeros(Nx,1);
y = zeros(Nx,1);
%% State equation
dx(1) =  p1 * u - p1*p2 * g(x(1)); 

for i = 2:Nx %:Nx-1
     dx(i) =  p1*p2*(g(x(i-1)) - g(x(i)));  
end

%% Output equations
y(1:Nx-1) = x(1:Nx-1);
y(Nx) = p2*g(x(Nx));

%% g(z) non-linear function 
function y = g(z)    
    y = ((z).^(5/3)) ./ ((z + p3).^(2/3));        
end
end