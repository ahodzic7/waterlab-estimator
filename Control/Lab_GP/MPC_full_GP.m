function [output]  = MPC_full_GP(X0,time)

% define persistent variables
eml.extrinsic('evalin');
persistent warmstartEnabler; persistent x_init;         persistent lam_g;
persistent OCP;              persistent D_sim;          persistent sigma_X0; 
persistent GP;               persistent inv_K_xx_val;   persistent u_prev;
persistent X_ref_sim;        persistent Hp;             persistent Z_train_subset;
persistent Y_train_subset;   persistent M;              persistent Nx

if isempty(lam_g)                % get persistent values from workspace
    warmstartEnabler = evalin('base','warmstartEnabler');
    lam_g = evalin('base','lam_g');
    x_init = evalin('base','x_init');
    OCP = evalin('base','OCP'); 
    D_sim = evalin('base','D_sim');
    sigma_X0 = evalin('base','sigma_X0');
    GP = evalin('base','GP');
    inv_K_xx_val = evalin('base','inv_K_xx_val');
    X_ref_sim = evalin('base','X_ref_sim');
    Hp = evalin('base','Hp');
    Z_train_subset = evalin('base','z_train_subset');
    Y_train_subset = evalin('base','y_train_subset');
    M = evalin('base','M');
    u_prev = [0;0];
    Nx = evalin('base','Nx');
end

dT = 1/6;                        % Sample time in minutes             
simulink_frequency = 2;          % Sampling frequency in seconds
time = int64(round(time));       % average time
disturbance = zeros(3,Hp);       % preallocation

% Disturbance calc.
for i=0:1:Hp-1
    start_index = time+1 + i*dT*60*simulink_frequency;
    end_index = start_index + dT*60*simulink_frequency-1;
    disturbance(:,i+1) = mean(D_sim(:,start_index:end_index),2);
end
% Reference calc.
reference = zeros(2,Hp);
for i=0:1:Hp-1
    start_index = time+1 + i*dT*60*simulink_frequency;
    end_index = start_index + dT*60*simulink_frequency-1;
    reference(:,i+1) = mean((X_ref_sim(:,start_index:end_index)),2);
end
% State measure 
X0 = X0/100;

tic
% openloop MPC
if warmstartEnabler == 1
    % Parametrized Open Loop Control problem with WARM START
    [U_opt_Hp,mu_X_opt,sigma_X_opt,lam_g,x_init] = ...
     OCP(X0,disturbance,sigma_X0,Z_train_subset,Y_train_subset,GP.sigma_f,GP.sigma_L,lam_g,x_init,reference,inv_K_xx_val,u_prev);
elseif warmstartEnabler == 0
    % Parametrized Open Loop Control problem without WARM START 
    [U_opt_Hp,mu_X_opt,sigma_X_opt] = ...
     OCP(X0,disturbance,sigma_X0,Z_train_subset,Y_train_subset,GP.sigma_f,GP.sigma_L,reference,inv_K_xx_val,u_prev);
end
toc

% Solution
u_sol = full(U_opt_Hp(:,1));
u_prev = u_sol;
Z_pred = [full(mu_X_opt); full(U_opt_Hp); disturbance];

% Subset of Data (SoD) point selection 
[Z_train_subset, Y_train_subset] = reduce_M(Z_pred,GP.z_train,GP.y_train,Hp,M);

% Pre-calculate K_xx and inv_K_xx
inv_K_xx_val = K_xx_builder(Z_train_subset,GP,Nx,M);

output = [u_sol; reference(:,1)*100];

end
