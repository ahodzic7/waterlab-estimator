function [dx, y] = model_cont_diff_wave(t, x, u, p1, p2, p3, p4, p5, Nx, varargin)
% Continuous time nlgreyest model for the kinematic wave level-based model. 

dx = zeros(Nx,1);
y = zeros(Nx+1,1);
%% State equation
dx(1) =  p1 * u - p1*p2 * x(1) + p1*p3*x(2)-p1*p4; 

for i = 2:Nx-1 %:Nx-1
     dx(i) =  p1*p2*x(i-1) - p1*(p2+p3)*x(i) + p1*p3*x(i+1);  
end

% Linear discharge (comment out one)
dx(Nx) = p1*p2 * x(Nx-1) - p1*p3*x(Nx) + p1*p4 -  p5*(x(Nx));
% Nonlinear discharge
%dx(Nx) = p1*p2 * x(Nx-1) - p1*p3*x(Nx) + p1*p4 -  p5*(x(Nx)^(2/3));
%% Output equations
y(1:Nx) = x(1:Nx);
y(Nx+1) = p5/p1*(x(Nx));        %linear
%y(Nx+1) = p5/p1*(x(Nx)^(2/3));  %nonlinear

end