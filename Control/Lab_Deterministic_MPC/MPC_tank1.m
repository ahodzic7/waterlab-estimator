function u  = MPC_tank1(X0,disturbance)
% define persistent variables
eml.extrinsic('evalin');
persistent lam_g;
persistent OCP;
persistent warmStartEnabler;
persistent x_init;
% and others
dt = 1/6;           % Sample time in minutes
u = 0;
% init persistent variables

if isempty(lam_g)
    lam_g = 1;
    x_init = 0;
    % get optimization problem and warmStartEnabler
    OCP = evalin('base','OCP'); 
    warmStartEnabler = 0;
    warmStartEnabler = evalin('base','warmStartEnabler');
end

% run openloop MPC
if warmStartEnabler == 1
    % Parametrized Open Loop Control problem with WARM START
    [u , S, lam_g, x_init] = (OCP(X0, disturbance, lam_g, x_init, dt));
elseif warmStartEnabler == 0
    % Parametrized Open Loop Control problem without WARM START 
    [u , S] = (OCP(X0, disturbance, dt));
end

u_full = full(u)
u = u_full(1);
end
