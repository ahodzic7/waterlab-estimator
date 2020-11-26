function [dx, y] = backflow_more_states(t, x, u, p1, p2, p3, p4, p5, p6,N_states , N_optimization_variables, N_aug_states, varargin)
% Continous time nlgreyest model for the diffusion wave gravity pipe with the tank in the end of the pipe. 
tank_offset = 0;
dx = zeros(N_states+N_aug_states,1);
y = zeros(N_optimization_variables,1);
N = N_aug_states/2;
%% State equations
dx(1) =  p1 * u(1) - p2 * x(1) + p3*x(2)-p4; 

dx(2) =  p2*x(1) - (p2+p3)*x(2) + p3*x(3);
dx(N_states+1) = p2*x(2) - (p2+p3)*x(N_states+1) + p3*x(N_states+2);
for j = 2:N-1
        dx(N_states + j) = p2*x(N_states+j-1) - (p2+p3)*x(N_states + j) + p3*x(N_states + j+1);
end
dx(N_states + N) = p2*x(N_states+N-1) - (p2+p3)*x(N_states + N) + p3*x(3);
dx(3) =  p2*x(N_states + N) - (p2+p3)*x(3) + p3*x(N_states+N+1);
dx(N_states+N+1) = p2*x(3) - (p2+p3)*x(N_states+N+1) + p3*x(N_states+N+2);
for j = 2:N-1
        dx(N_states + N+j) = p2*x(N_states+N+j-1) - (p2+p3)*x(N_states +N+ j) + p3*x(N_states +N+ j+1);
end
dx(N_states + 2*N) = p2*x(N_states+2*N-1) - (p2+p3)*x(N_states + 2*N) + p3*x(4);
%% Last pipe equation
%if N_states > 2
% dx(N_states-1) = p2 * x(N_states-2) - p3*x(N_states-1) + p4 -  p5*(x(N_states-1));
% Submerged flow (comment out)
dx(N_states-1) = p2 * x(N_states-2) - p3*x(N_states-1)+ p4 -  p5*(x(N_states-1)-x(N_states)+tank_offset);

%% Tank equation
% dx(N_states) = p6*(p5/p1*(x(N_states-1))-u(2));
% Tank with Submerged flow (comment out)
dx(N_states) = p6*(p5/p1*(x(N_states-1)-x(N_states)+tank_offset)-u(2));


%% Output equation
y(1:N_states) = x(1:N_states);
if N_optimization_variables > N_states
    y(N_optimization_variables) = p5/p1*(x(N_states-1));
    %y(N_optimization_variables) = p5/p1*(x(N_states-1)-x(N_states)+tank_offset);        %submerged flow
end

end