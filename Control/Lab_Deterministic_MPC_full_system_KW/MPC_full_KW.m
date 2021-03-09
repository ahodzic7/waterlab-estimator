function [output]  = MPC_full_KW(X0,time)
% define persistent variables
eml.extrinsic('evalin');
persistent x_init;
persistent lam_g;
persistent OCP;
persistent P_sim;
persistent D_sim;

persistent Hp;
persistent warmStartEnabler;
persistent log;

% and others
dT = 1/6;           % Sample time in minutes
simulink_frequency = 2;  % Sampling frequency in seconds
% init persistent variables

if isempty(lam_g)
    lam_g = 1;
    x_init = 0;
    Hp = 0;
    P_sim = zeros(5,1);
    warmStartEnabler = 0;
    % get optimization problem and warmStartEnabler
    OCP = evalin('base','OCP');
    Hp = evalin('base','Hp');
    D_sim = evalin('base','D_sim');
    P_sim = evalin('base','P_sim');
    warmStartEnabler = evalin('base','warmStartEnabler');
end

time = int64(round(time));
disturbance = zeros(3,Hp);
for i=0:1:Hp-1
    start_index = time+1 + i*dT*60*simulink_frequency;
    end_index = start_index + dT*60*simulink_frequency-1;
    disturbance(:,i+1) = mean(D_sim(:,start_index:end_index),2);
end

% Unit convertion:

X0 = X0/100;
reference = [3;3.5];

% run openloop MPC
if warmStartEnabler == 1
    % Parametrized Open Loop Control problem with WARM START
    [u,S,Y,lam_g,x_init] = OCP(X0, disturbance, P_sim, reference, lam_g, x_init, dT);
elseif warmStartEnabler == 0
    % Parametrized Open Loop Control problem without WARM START 
    [u,S,Y] = OCP(X0, disturbance, P_sim, reference, dT);
end


u_full = full(u);
S_full = full(S);
u = u_full(1:2);
S = S_full(1:2);

output = [u;S];

%plot_simulink_mpc(u_full,S_full,X0,disturbance,Hp);


end
