function fun_out=MPC_func(p_0_all,u_act)

persistent i C_con Hp opti p_0_par dc_par C_par u_var Pp_kalman Q_kalman R_kalman x_hat...
    phi_kalman_model C_kalman_model B_kalman_model Demand_C n_states_signal v1 v2

if isempty(x_hat)
    
    Hp=24;
    Ts=1;
    T_min=24*6;         % [min] Simulation time. Provide simulation time in multiple of 24 only.
    
    % Generating cost of electricity
    C_con_set=[0.7*ones(1,6),1.4*ones(1,14),0.7*ones(1,4)];
    C_con=[];
    for i=1:(T_min+Hp)/24
        C_con=horzcat(C_con,C_con_set);
    end
    
    %% Loading parameters from workspace
    parameters_loading
    
    %% Defining NMPC problem
    NMPC_problem_casadi
    
    %% Defining parameters for Kalman filter
    tau_s=60;
    Pp_kalman=100*diag(1*ones(n_states_signal+1,1));               %Autocovariance for the initial estimate
    Q_kalman=(0.0351^2)*diag(ones(n_states_signal+1,1));%std_state_noise^2;                %Covariance of system noise
    Q_kalman(end,end)=Q_kalman(end,end)*(tau*tau_s)^2;
    R_kalman=0.00005^2;                    %Covariance of measurement noise
    
    x_hat=zeros(6,1);
    
    %% Initiate controller
    i=0;
    C_con=[0,C_con];
    fun_out=zeros(4,1);
end


i=i+1;
C_Hp=C_con(:,i:i+Hp-1);

%% Kalman filter

y=p_0_all;
%Predict state for next time step
x_hat= phi_kalman_model*x_hat+B_kalman_model*u_act;


[K,Pc,Pp_kalman]=kfgain_cal(phi_kalman_model,C_kalman_model,Pp_kalman,Q_kalman,R_kalman,n_states_signal+1);        %Calculate Kalman Gain and Covariance matrices

%Innovation Process
e=y-C_kalman_model*x_hat;

%Corrected state estimate
x_hat=x_hat+K*e;


D_esti=Demand_C*x_hat;

x_pred=zeros(6,24);
x_pred(:,1)=x_hat;

for a=1:24
    x_pred(:,a+1)= phi_kalman_model*x_pred(:,a);
end

D_predict=Demand_C*x_pred(:,2:end);

D_predict(D_predict>0)=0;
D_predict(D_predict<-0.45)=-0.45;

dc_Hp=[v1*D_predict;v2*D_predict];

p_esti=x_hat(6,1);

%% Setting values for NMPC
opti.set_value(p_0_par,p_esti);
opti.set_value(dc_par,dc_Hp);
opti.set_value(C_par,C_Hp);


sol = opti.solve();
u_all= sol.value(u_var);

u_out=u_all(:,1);



fun_out=[u_out',D_esti,p_esti];

