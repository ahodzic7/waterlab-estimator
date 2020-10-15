function [dx, y] = free_flow_model(t, x, u, p1, p2, p3, p4, p5, p6, Nx, varargin)
% Continous time nlgreyest model for the kinematic wave level-based model. 

tank_area = 1.25^2*pi; %[dm^2]

dx(1) =  p1 * u(1) - p2 * x(1) + p3*x(2)-p4; 

for i = 2:Nx-2 %:Nx-1
     dx(i) =  p2*x(i-1) - (p2+p3)*x(i) + p3*x(i+1); 
     %disturbace model:
     %( a1* cos(w*t) + b1* sin(w*t) + a2* cos(2*w*t) + b2* sin(2*w*t)+...
     %    a3* cos(3*w*t) + b3* sin(3*w*t)+ a4* cos(4*w*t) + b4* sin(4*w*t));
end

% Linear discharge free flow(comment out one)
dx(Nx-1) = p2 * x(Nx-2) - p3*x(Nx-1) + p4 -  p5*(x(Nx-1));
dx(Nx) = p6*((p5/p1)*x(Nx-1)-u(2));             %Tank equation

y(1:Nx) = x(1:Nx);      

end