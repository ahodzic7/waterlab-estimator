function [dx, y] = free_flow_model_augmented_2(t, x, u, p1, p2, p3, p4, p5, p6,N_states , N_optimization_variables, N_aug_states, varargin)
% Continous time nlgreyest model for the diffusion wave gravity pipe with the tank in the end of the pipe. 

dx = zeros(N_states+N_aug_states,1);
y = zeros(N_optimization_variables,1);
%% Augmented states

dx(N_states + 1) = p1 * u(1) - p2 * x(N_states + 1) + p3*x(N_states + 2)-p4;
dx(N_states + 2) = p2*x(N_states+1) - (p2+p3)*x(N_states + 2) + p3*x(1);

%% State equations
dx(1) =  p2*x(N_states+2) - (p2+p3)*x(1) + p3*x(N_states+3);

dx(N_states + 3) = p2*x(1) - (p2+p3)*x(N_states + 3) + p3*x(N_states + 4);
dx(N_states + 4) = p2*x(N_states+3) - (p2+p3)*x(N_states + 4) + p3*x(2);
dx(2) =  p2*x(N_states + 4) - (p2+p3)*x(2) + p3*x(N_states + 5); 

dx(N_states + 5) = p2*x(2) - (p2+p3)*x(N_states + 5) + p3*x(N_states + 6);
dx(N_states + 6) = p2*x(N_states + 5) - (p2+p3)*x(N_states + 6) + p3*x(3);
dx(3) =  p2*x(N_states + 6) - (p2+p3)*x(3) + p3*x(N_states + 7); 

if N_states > 2
    dx(N_states + 7) = p2*x(3) - (p2+p3)*x(N_states + 7) + p3*x(N_states + 8);
    dx(N_states + 8) = p2*x(N_states + 7) - (p2+p3)*x(N_states + 8) + p3*x(4);
    dx(N_states-1) = p2 * x(N_states+8) - p3*x(N_states-1) + p4 -  p5*(x(N_states-1));

%% Tank equation
dx(N_states) = p6*(p5/p1*(x(N_states-1))-u(2));

%% Output equation
y(1:N_states) = x(1:N_states);
if N_optimization_variables > N_states
    y(N_optimization_variables) = p5/p1*(x(N_states-1));
end

end