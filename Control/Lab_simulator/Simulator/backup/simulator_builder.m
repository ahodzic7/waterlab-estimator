%% casadi_builder.m
% """
% Builds optimization problem
% """
addpath('C:\Users\74647\OneDrive - Grundfos\MATLAB\CasAdi') 
import casadi.*
opti = casadi.Opti();                                       % Optimization problem

%% ============================================== Dynamics =====================================
% Runge-Kutta 4 integration 
dt_sym = casadi.MX.sym('dt',1);  
x_sym = casadi.MX.sym('x',Nxt+ Nxp);
u_sym = casadi.MX.sym('u',Nxt);
d_sym = casadi.MX.sym('d',ND);                       % [d_t1, d_t2, d_p]
p_sym = casadi.MX.sym('p',NP);                    
%
k1_sym = dynamics_sim(x_sym, u_sym, d_sym, p_sym);
k2_sym = dynamics_sim(x_sym + dt_sym / 2.0 * k1_sym, u_sym, d_sym, p_sym);
k3_sym = dynamics_sim(x_sym + dt_sym / 2.0 * k2_sym, u_sym, d_sym, p_sym);
k4_sym = dynamics_sim(x_sym + dt_sym * k3_sym, u_sym, d_sym, p_sym);

intMethod = 1;
if intMethod == 1
    xf = x_sym + dt_sym / 6.0 * (k1_sym + 2 * k2_sym + 2 * k3_sym + k4_sym);
    F_integral_sim = casadi.Function('F_RK4', {x_sym, u_sym, d_sym, p_sym, dt_sym}, {xf}, {'x[k]','u[k]','d[k]','p','dt'},{'x[k+1]'});
elseif intMethod == 2
    xf = x_sym + dt_sym*dynamics_sim(x_sym, u_sym, d_sym, p_sym);
    F_integral_sim = casadi.Function('F_EUL', {x_sym, u_sym, d_sym, p_sym, dt_sym}, {xf}, {'x[k]','u[k]','d[k]','p','dt'},{'x[k+1]'});
end

%%
function y = g(z,p3)    
    y = ((z).^(5/3)) ./ (((z) + p3).^(2/3));        
end
