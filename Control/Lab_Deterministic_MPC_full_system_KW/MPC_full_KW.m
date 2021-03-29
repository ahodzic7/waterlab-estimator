function [output]  = MPC_full_KW(X0,time)

% define persistent variables
eml.extrinsic('evalin');
persistent x_init;
persistent lam_g;
persistent OCP;
persistent P_sim;
persistent D_sim;
persistent X_ref_sim;
persistent Hp;
persistent warmStartEnabler;

% Sample time in minutes
dT = 1/6;             
% Sampling frequency in seconds
simulink_frequency = 2;  

% init persistent variables
if isempty(lam_g)
    lam_g = 1;
    x_init = 0.001;
    Hp = 0;
    P_sim = zeros(5,1);
    warmStartEnabler = 1;
    
    % get values from workspace
    OCP = evalin('base','OCP');
    Hp = evalin('base','Hp');
    D_sim = evalin('base','D_sim');
    X_ref_sim = evalin('base','X_ref_sim');
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

reference = zeros(2,Hp);
for i=0:1:Hp-1
    start_index = time+1 + i*dT*60*simulink_frequency;
    end_index = start_index + dT*60*simulink_frequency-1;
    reference(:,i+1) = mean((X_ref_sim(:,start_index:end_index)),2);
end

X0 = X0/100;

% openloop MPC
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

output = [u;S;reference(:,1)*100];

%plot_simulink_mpc(u_full,S_full,X0,disturbance,Hp);

end
