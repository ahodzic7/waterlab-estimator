SMPC_init_DW;

%% ====================================== Simulation ==================================


%Prep for sim and plot
N=200;
dT = 10;
% number of simulation steps
X_sim = casadi.DM.zeros(nS, N+1); 
X_sim(:,1) = [4 0.01 0.01 0.01 0.01 4];
X_sim_num = full(X_sim);
X_predict = casadi.DM.zeros(nS, Hp+1);
reference = [3 0.01 0.01 0.01 0.01 3];

U_sim = casadi.DM.zeros(nU, N+1);
U_sim(:,1) = U_ub;
U_sim_num = full(U_sim);

S_sim = casadi.DM.zeros(nU, N+1); 
S_ub_sim = casadi.DM.zeros(nT, N+1);

make_LQR      % Calculate LQR
make_sigma_X  % Precompute sigma_X for chance constraint, Open Loop MPC

%Set up sim disturbance:
disturbance = D_sim(1:2:3,1:N+Hp)/60;
%disturbance(60:90) = 0.4;
dist_forcast = disturbance + normrnd(0,0.1,size(disturbance))/60;


%Variables for warmstart
lam_g = 1;
x_init = 0.01;

%Run Closed loop mpc
for step = 1:1:N
    %Open loop predicition
    if warmStartEnabler == 1
        % Parametrized Open Loop Control problem with WARM START
        [u , s, s_ub, lam_g, x_init] = OCP(X_sim(:,step),U_sim(:,step),dist_forcast(:,step:1:step+Hp-1), lam_g, x_init, dT, reference,sigma_x);
    elseif warmStartEnabler == 0
        % Parametrized Open Loop Control problem without WARM START 
        [u , s, s_ub] = (OCP(X_sim(:,step),U_sim(:,step),dist_forcast(:,step:1:step+Hp-1), dT, reference,sigma_x));
    end

    %Predict comming states:
    X_predict(:,1) = X_sim(:,step);
    for i = 1:Hp
        X_predict(:,i+1) = F_system(X_predict(:,i), u(:,1) + s(:,i), dist_forcast(:,step+i-1), dT);
    end
    
    %Get numerical value
    U_out_num = full(u);
    S_out_num = full(s);
    S_ub_out_num = full(s_ub);
    X_predict_num = full(X_predict);
    U_sim_num = full(U_sim);
    S_sim_num = full(S_sim);
   	S_ub_sim_num = full(S_ub_sim);
    
    plot_MPC;
    
    %Advance simulation and save values
    X_sim(:,step+1) = F_system(X_sim(:,step), u(:,1), disturbance(:,step), dT);
    U_sim(:,step+1) = u(:,1);
    S_sim(:,step+1) = s(:,1);
    S_ub_sim(:,step+1) = s_ub(:,1);
    
    X_sim_num = full(X_sim);
    for i = 1:nS
        if X_sim_num(i,step+1) > X_ub(i,1) 
           X_sim(i,step+1) = X_ub(i,1);
           X_sim_num(i,step+1) = X_ub(i,1);
        elseif X_sim_num(i,step+1) < X_lb(i,1)
           X_sim(i,step+1) = X_lb(i,1);
           X_sim_num(i,step+1) = X_lb(i,1);
        end
    end
end
