function [output]  = SMPC_full_DW(X0,time)
% define persistent variables
eml.extrinsic('evalin');
persistent x_init;
persistent lam_g;
persistent OCP;
persistent Hp;
persistent warmStartEnabler;
persistent D_sim;
persistent X_ref;
persistent U0;
persistent sigma_x;
persistent X_pre;
persistent sys;
persistent K;
% and others
dT = 10;                 % Sample time in minutes
simulink_frequency = 2;  % Sampling frequency in seconds

if isempty(lam_g) 
    % init persistent variables
    lam_g = 1;
    x_init = 0.001;
    
    % initialize MPC
    U0 = [3;4.5];
    X_pre = X0/100;
    
    % get optimization problem and warmStartEnabler
    OCP = evalin('base','OCP');
    Hp = evalin('base','Hp');
    D_sim = evalin('base','D_sim');
    D_sim = D_sim(1:2:3,:);
    X_ref = evalin('base','X_ref_sim');
    warmStartEnabler = evalin('base','warmStartEnabler');
    
    % initialize CC tube controller
    make_LQR      % Calculate LQR
    make_sigma_X  % Precompute sigma_X for chance constraint, Open Loop MPC
end


%Create forcast from disturbance reference
time = int64(round(time));
disturbance = zeros(2,Hp);
for i=0:1:Hp-1
    start_index = time+1+i*dT*simulink_frequency;
    end_index = start_index+dT*simulink_frequency-1;
    disturbance(:,i+1) = mean(D_sim(:,start_index:end_index),2)/60;
end

X0 = X0/100;                                                               % Unit convertion from mm to dm
reference = [X_ref(1,time+1), 0, 0, 0, 0, X_ref(2,time+1)]';

% run openloop MPC
if warmStartEnabler == 1
    % Parametrized Open Loop Control problem with WARM START
    [u , S, S_ub, lam_g, x_init] = (OCP(X0,U0,disturbance, lam_g, x_init, dT,reference,sigma_x));
elseif warmStartEnabler == 0
    % Parametrized Open Loop Control problem without WARM START 
    [u , S, S_ub] = (OCP(X0 ,U0, disturbance, dT, reference,sigma_x));
end

% create outputs
u_full = full(u);
S_full = full(S);
S_ub_full = full(S_ub);
(X0-X_pre)
lqr_contribution =min(1/60*ones(2,1), max(-1/60*ones(2,1),  K*(X0-X_pre)));
lqr_contribution*60

output = [min(sys.U_ub, max(sys.U_lb, u_full(:,1) - lqr_contribution)) ; S_full(:,1)]*60;           % Saturate the outputs
output = [output; X_ref(:,time+1)*100; S_ub_full(:,1)];

% Set vairables for next iteration
U0 = u_full(:,1);
X_pre = full(sys.F_system(X0, u_full(:,1), disturbance(:,1), dT))
for i =1:5:6
    if X_pre(i) > sys.X_ub(i)
        X_pre(i) = sys.X_ub(i);
    elseif X_pre(i) < sys.X_lb(i)
        X_pre(i) = sys.X_lb(i);
    end
end
end
