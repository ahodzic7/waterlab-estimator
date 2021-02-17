API reference for functions

[SystemStruct]=setup_linear_uncertain_system_model_single_tank()
% x(n+1) = A x(n) + B u + B_d (u_d_known + u_disturbance) + model_uncertainty

[SystemStruct]=setup_affine_linear_uncertain_system_model_aau_smart_water_lab()

[SystemStruct]=setup_affine_linear_uncertain_system_model_simple_model()

[SystemStruct]=setup_affine_linear_uncertain_system_model_fredericia()
% % x(n+1) = A x(n) + B u + B_d (u_d_known + u_disturbance) + model_uncertainty + Delta

[ConstraintsStruct] = Generate_Constraints([X_lb X_ub], [U_lb U_ub], [dU_lb dU_ub], Hp, Hu=Null)

[ObjectiveStruct] = Generate_Linear_Objective(Q, R=Null, S=Null, ObjectiveStruct=Null)

[ObjectiveStruct] = Generate_Quadratic_Objective(Q, R==Null, S==Null, ObjectiveStruct=Null)

[solverObj] = setup_MPC_solver(SystemStruct,ConstraintsStruct,ObjectiveStruct)

[U_predict, S_predict] = MPC_solve(solverObj, X0, disturbance_forecast)

% Data types:
SystemStruct
{
A                       Size: Number_of_states x Number_of_states
B                       Size: Number_of_states x Number_of_inputs
B_disturbance           Size: Number_of_states x Number_of_disturbances
C                       Size: Number_of_outputs x Number_of_states
Delta                   Size: Number_of_states x 1
prediction_uncertainty  Assumed gaussian, [mean, variance]
model_uncertainty       Assumed gaussian, [mean, variance]
}

ConstraintsStruct
{

}

ObjectiveStruct
{

}