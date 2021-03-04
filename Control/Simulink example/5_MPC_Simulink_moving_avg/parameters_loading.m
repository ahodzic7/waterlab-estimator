%% Paramters for NMPC problem and Kalman filter from workspace

    M_p_bar = evalin('base', 'M_p_bar');
    M_c_bar = evalin('base', 'M_c_bar');
    M_tau_bar = evalin('base', 'M_tau_bar');
    
    rho_fluid = evalin('base', 'rho_fluid');
    g = evalin('base', 'g');
    
    n_H = evalin('base', 'n_H');
    edge_tree = evalin('base', 'edge_tree');
    H_T_bar = evalin('base', 'H_T_bar');
    H_C_bar = evalin('base', 'H_C_bar');
    B = evalin('base', 'B');
    
    
    lambda = evalin('base', 'lambda');
    h_bar = evalin('base', 'h_bar');
    h_0 = evalin('base', 'h_0');
    
    tau = evalin('base', 'tau');
        
    phi_kalman_model=evalin('base', 'phi_kalman_model');
    C_kalman_model=evalin('base', 'C_kalman_model');
    B_kalman_model=evalin('base', 'B_kalman_model');
    n_states_signal=evalin('base', 'n_states_signal');
    Demand_C=evalin('base', 'Demand_C');
    
    v1=evalin('base', 'v1');
    v2=evalin('base', 'v2');