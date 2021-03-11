function [dx, y] = free_flow_model(t, x, u, p1, p2, p3, p4, p5,N_states , N_optimization_variables, N_aug_states, varargin)
% Continous time nlgreyest model for the diffusion wave gravity pipe with the tank in the end of the pipe. 

dx = zeros(N_states,1);
y = zeros(N_optimization_variables,1);

%% State equations
dx(1) =  p1 * u(1) - p2 * x(1) + p3*x(2)-p4; 

for i = 2:N_states-1
     dx(i) =  p2*x(i-1) - (p2+p3)*x(i) + p3*x(i+1); 
end

%% Last pipe equation
dx(N_states) = p2 * x(N_states-1) - p3*x(N_states) + p4 -  p5*(x(N_states));


%% Output equation
y(1:N_states) = x(1:N_states);
y(N_optimization_variables) = p5/p1*(x(N_states));
end