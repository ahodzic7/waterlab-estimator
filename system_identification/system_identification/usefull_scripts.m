plot(estParams(5)/estParams(1)*(h(N_states-1,:)));
t = 0:0.5:988/2-0.5;
dTdt = gradient(T2(:)) ./ gradient(t(:));

dTdt'/estParams(6) + Q(2,:)

%%
input = [Q(1,:)' Q(2,:)' h(2,:)' h(3,:)' h(4,:)'];

function [dx, y] = free_flow_model(t, x, u, p1, p2, p3, p4, p5, p6,N_states , N_optimization_variables, varargin)
% Continous time nlgreyest model for the diffusion wave gravity pipe with the tank in the end of the pipe. 

dx = zeros(N_states,1);
y = zeros(N_optimization_variables,1);

%% State equations
dx(1) =  p1 * u(1) - p2 * x(1) + p3*u(3)-p4; 

dx(2) = p2*x(1) - (p2+p3)*x(2) + p3*u(4);
dx(3) = p2*u(3) - (p2+p3)*x(3) + p3*u(5);
dx(4) = p2 * u(4) - p3*x(4) + p4 -  p5*(x(4));
dx(5) = p6*(p5/p1*(u(4))-u(2));

% for i = 2:N_states-2
%      dx(i) =  p2*x(i-1) - (p2+p3)*x(i) + p3*x(i+1); 
% end
% 
% if N_states > 2
% dx(N_states-1) = p2 * x(N_states-2) - p3*x(N_states-1) + p4 -  p5*(x(N_states-1));
% 
% %% Tank equation
% dx(N_states) = p6*(p5/p1*(x(N_states-1))-u(2));

%% Output equation
y(1:N_states) = x(1:N_states);
if N_optimization_variables > N_states
    y(N_optimization_variables) = p5/p1*(x(N_states-1));
end

end