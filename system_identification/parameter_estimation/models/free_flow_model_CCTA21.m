function [dx, y] = free_flow_model_CCTA21(t, x, u, p1, p2, p3, p4, p5, p6, N_states, N_optimization_variables, N_aug_states, varargin)
% Continous time nlgreyest model for the diffusion wave gravity pipe with the tank in the end of the pipe. 

dx = zeros(N_states+N_aug_states,1);
y = zeros(N_optimization_variables,1);

%% Define State Equations

%State 1 - Pump Inflow
dx(1) =  p1 * u(1) - p2 * x(1) + p3*x(2) - p4;

%State 2 to 4 - Pipe equation
for i = 2:4
     dx(i) =  p2*x(i-1) - (p2+p3)*x(i) + p3*x(i+1); 
end

%State 5 - Pipe equation with lateral inflow u(3)
dx(5) =  p2*x(4) - (p2+p3)*x(5) + p3*x(6) + p1*u(3); 

%State 6 to 7 - Pipe equation
for i = 6:7
     dx(i) =  p2*x(i-1) - (p2+p3)*x(i) + p3*x(i+1); 
end

%State 8 - Free flow boundery equation
dx(8) = p2 * x(7) - (p3 + p5)*x(8) + p4;

%State 9 - Tank equation
dx(9) = p6*(p5/p1*(x(8))-u(2));

%% Output equation
y(1:N_states) = [x(1,2);
                 x(1,4);
                 x(1,5);
                 x(1,7);
                 x(1,9)];
end