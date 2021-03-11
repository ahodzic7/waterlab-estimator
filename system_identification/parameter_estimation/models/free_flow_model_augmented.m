function [dx, y] = free_flow_model_augmented(t, x, u, p1, p2, p3, p4, p5,N_states , N_optimization_variables, N_aug_states, varargin)
% Continous time nlgreyest model for the diffusion wave gravity pipe with the tank in the end of the pipe. 

dx = zeros(N_states+N_aug_states,1);
y = zeros(N_optimization_variables,1);
N = N_aug_states/4;
%% Augmented states
if N > 1
    dx(N_states + 1) = p1 * u(1) - p2 * x(N_states + 1) + p3*x(N_states + 2)-p4;
    for j = 2:N-1
        dx(N_states + j) = p2*x(N_states+j-1) - (p2+p3)*x(N_states + j) + p3*x(N_states + j+1);
    end
    dx(N_states + N) = p2*x(N_states+N-1) - (p2+p3)*x(N_states + N) + p3*x(1);
else
    dx(N_states + 1) = p1 * u(1) - p2 * x(N_states + 1) + p3*x(1)-p4;
end

%% State equations
dx(1) =  p2*x(N_states+N) - (p2+p3)*x(1) + p3*x(N_states+N+1);

if N > 1
    dx(N_states +N+ 1) = p2*x(1) - (p2+p3)*x(N_states + N+1) + p3*x(N_states+N + 2);
    for j = 2:N-1
        dx(N_states+N + j) = p2*x(N_states+N+j-1) - (p2+p3)*x(N_states+N + j) + p3*x(N_states+N + j+1);
    end
    dx(N_states + 2*N) = p2*x(N_states+2*N-1) - (p2+p3)*x(N_states + 2*N) + p3*x(2);
else
    dx(N_states + 2) = p2*x(1) - (p2+p3)*x(N_states + 2) + p3*x(2);
end
dx(2) =  p2*x(N_states + 2*N) - (p2+p3)*x(2) + p3*x(N_states + 2*N+1); 

if N > 1
    dx(N_states +2*N+ 1) = p2*x(2) - (p2+p3)*x(N_states + 2*N+1) + p3*x(N_states+2*N + 2);
    for j = 2:N-1
        dx(N_states+2*N + j) = p2*x(N_states+2*N+j-1) - (p2+p3)*x(N_states+2*N + j) + p3*x(N_states+2*N + j+1);
    end
    dx(N_states + 3*N) = p2*x(N_states+3*N-1) - (p2+p3)*x(N_states + 3*N) + p3*x(3);
else
    dx(N_states + 3) = p2*x(2) - (p2+p3)*x(N_states + 3) + p3*x(3);
end
dx(3) =  p2*x(N_states + 3*N) - (p2+p3)*x(3) + p3*x(N_states + 3*N+1); 

if N > 1
    dx(N_states +3*N+ 1) = p2*x(3) - (p2+p3)*x(N_states + 3*N+1) + p3*x(N_states+3*N + 2);
    for j = 2:N-1
        dx(N_states+3*N + j) = p2*x(N_states+3*N+j-1) - (p2+p3)*x(N_states+3*N + j) + p3*x(N_states+3*N + j+1);
    end
    dx(N_states + 4*N) = p2*x(N_states+4*N-1) - (p2+p3)*x(N_states + 4*N) + p3*x(4);
else
    dx(N_states + 4) = p2*x(3) - (p2+p3)*x(N_states + 4) + p3*x(4);
end
dx(N_states) = p2 * x(N_states+4*N) - p3*x(N_states) + p4 -  p5*(x(N_states));


%% Output equations
y(1:N_states) = x(1:N_states);
y(N_optimization_variables) = p5/p1*(x(N_states));

end