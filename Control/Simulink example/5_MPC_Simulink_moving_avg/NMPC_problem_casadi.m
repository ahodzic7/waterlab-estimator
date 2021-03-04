%% NMPC problem in Casadi
  
opti = casadi.Opti();

%% Constant and contraint values

M_p_bar_inv=pinv(M_p_bar);  % Transformation matrix to extract pressure at pumping station
M_c_bar_inv=pinv(M_c_bar);  % Transformation matrix to extract pressure at consumer nodes

eta_p=0.6;                          % Efficiency of pumps
eta_m=0.9;                          % Efficiency of motors
k_eta=1/(eta_p*eta_m*1e3*1e5*3600); % Constant based on efficiency

% Limits on different variables
h0_min=0.1;                         % [m] Minimum level in tank
h0_max=0.4;                         % [m] Max level in tank
p0_min=h0_min*rho_fluid*g/1e5;
p0_max=h0_max*rho_fluid*g/1e5;
u_min=[0;0];
u_max=[0.3;0.3];                        % [m^3/h] Max flow from pumping station
p_p_min=[0;0];
p_p_max=[0.6;0.6];                  % [bar] Max pressure from pumping station

%% MPC weights

Q_weight=5.5e10*diag([1 1]);
R_weight=1*diag([1 1]);
Rho_weight=1e5;

%% Casadi variables

% Variables in the NLP problem
u_var=opti.variable(2,Hp);

qC_var=opti.variable(2,Hp);
p_bar_var=opti.variable(5,Hp);
p_0_var=opti.variable(1,Hp+1);

eps_var=opti.variable(1,Hp);

% Parameters in the NLP problem

p_0_par=opti.parameter(1,1);

dc_par=opti.parameter(2,Hp);
C_par=opti.parameter(1,Hp);

obj=0;

for k=1:Hp
    
    % Objective function for MPC
    
    C_val=diag([C_par(:,k) C_par(:,k)]);
    p_p_val=M_p_bar_inv*p_bar_var(:,k);
    
    
    obj=obj+u_var(:,k)'*Q_weight*C_val*p_p_val*k_eta*Ts;
    obj=obj+(M_c_bar_inv*(p_bar_var(:,k)-mean(p_bar_var(:,k))))'*R_weight*(M_c_bar_inv*(p_bar_var(:,k)-mean(p_bar_var(:,k))));
    obj=obj+Rho_weight*eps_var(:,k);
    
    % System constraitns for MPC
    
    d_tau_val=-(sum(dc_par(:,k))+sum(u_var(:,k)));
    
    qT_val=-inv(H_T_bar)*H_C_bar*qC_var(:,k)+inv(H_T_bar)*M_p_bar*u_var(:,k)...
        +inv(H_T_bar)*M_c_bar*dc_par(:,k)+inv(H_T_bar)*M_tau_bar*d_tau_val;
    
    q_val=[qC_var(1,k);qT_val(1:2,1);qC_var(2,k);qT_val(3:5,1)];
    
    lambda_q_val=lambda.*abs(q_val).*q_val;
    
    st_1=B*lambda_q_val;
    st_2=inv(H_T_bar')*lambda_q_val(edge_tree)-(h_bar-ones(n_H-1,1)*h_0)+ones(n_H-1,1)*p_0_var(:,k);
    st_3=p_0_var(:,k)-tau*d_tau_val*60;
    
    opti.subject_to(st_1==0);
    opti.subject_to(p_bar_var(:,k)==st_2);
    opti.subject_to(p_0_var(:,k+1)==st_3);
    
    % Inequality constraints
    
    opti.subject_to(u_min<=u_var(:,k));
    opti.subject_to(u_var(:,k)<=u_max);
    
    opti.subject_to(p_p_min<=p_p_val);
    opti.subject_to(p_p_val<=p_p_max);
    
    opti.subject_to((p0_min-eps_var(:,k))<=p_0_var(:,k+1));
    opti.subject_to(p_0_var(:,k+1)<=(p0_max+eps_var(:,k)));
    
    opti.subject_to(0<=eps_var(:,k));
    
end

opti.subject_to(p_0_var(:,1)==p_0_par);

opts=struct;
opts.ipopt.print_level=0;

opti.solver('ipopt',opts);


opti.minimize(obj);
