function [dx, y] = altered_free_flow_model(t, x, u, p1, p2, p3, p4, p5, p6,N_states , N_optimization_variables,p9,p10, varargin)
% Continous time nlgreyest model for the diffusion wave gravity pipe with the tank in the end of the pipe. 

dx = zeros(N_states,1);
y = zeros(N_optimization_variables,1);

%% State equations
dx(1) =  p1 * u(1) - p2 * x(1) + p3*x(2)-p4; 


dx(2) =  p2*x(1) - (p2+p3)*x(2) + p9*x(3);
dx(3) =  p10*x(2) - (p2+p3)*x(3) + p3*x(4);


dx(4) = p2 * x(3) - p3*x(4) + p4 -  p5*(x(4));

%% Tank equation
dx(N_states) = p6*(p5/p1*(x(N_states-1))-u(2));

%% Output equation
y(1:N_states) = x(1:N_states);
if N_optimization_variables > N_states
    y(N_optimization_variables) = p5/p1*(x(N_states-1));
end

end