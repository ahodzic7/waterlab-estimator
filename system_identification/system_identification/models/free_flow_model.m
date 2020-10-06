function [dx, y] = free_flow_model(t, x, u, p1, p2, p3, p4, p5, Nx, varargin)
% Continuous time nlgreyest model for the kinematic wave level-based model. 
tank_offset = 1;
dx = zeros(Nx,1);
y = zeros(Nx+1,1);
%% State equation
dx(1) =  p1 * u(1) - p2 * x(1) + p3*x(2)-p4; 

for i = 2:Nx-1 %:Nx-1
     dx(i) =  p2*x(i-1) - (p2+p3)*x(i) + p3*x(i+1); 
     %disturbace model:
     %( a1* cos(w*t) + b1* sin(w*t) + a2* cos(2*w*t) + b2* sin(2*w*t)+...
     %    a3* cos(3*w*t) + b3* sin(3*w*t)+ a4* cos(4*w*t) + b4* sin(4*w*t));
end

% Linear discharge free flow(comment out one)
dx(Nx) = p2 * x(Nx-1) - p3*x(Nx) + p4 -  p5*(x(Nx));
% Submerged flow (comment out)
% dx(Nx) = p1*p2 * x(Nx-1) - p1*p3*x(Nx)+ p1*p4 -  p5*(x(Nx)-u(2)+tank_offset);
% Offset flow
%dx(Nx) = p1*p2 * x(Nx-1) - (p2+p3)*x(Nx) + p1*p3*(u(2)-tank_offset);
% Nonlinear discharge
%dx(Nx) = p1*p2 * x(Nx-1) - p1*p3*x(Nx) + p1*p4 -  p5*(x(Nx)^(2/3));
%% Output equations
y(1:Nx) = x(1:Nx);
y(Nx+1) = p5/p1*(x(Nx));        %linear free flow
%y(Nx+1) = p5/p1*(x(Nx)-u(2)+tank_offset);        %submerged flow
%y(Nx+1) = p2*x(Nx)-p3*(u(2)-tank_offset)+p4;                   %offset flow
%y(Nx+1) = p5/p1*(x(Nx)^(2/3));  %nonlinear

end