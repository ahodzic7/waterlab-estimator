function [SystemStruct]=setup_affine_linear_uncertain_system_model_simple_model()
% SystemStruct:
% {
    % A                       Size: Number_of_states x Number_of_states
    % B                       Size: Number_of_states x Number_of_inputs
    % B_disturbance           Size: Number_of_states x Number_of_disturbances
    % C                       Size: Number_of_outputs x Number_of_states
    % Delta                   Size: Number_of_states x 1
    % prediction_uncertainty  Assumed gaussian, [mean, variance]
    % model_uncertainty       Assumed gaussian, [mean, variance]
    % measurement_uncertainty Assumed gaussian, [mean, variance]
% }
    A = 1;
    B = 1;                              % inputs are flows
    B_disturbance = 1;                  % disturbances are flows
    C = 1;
    Delta = 0;
    
    prediction_uncertainty = struct('mu',0,'Sigma',0.001);    
    model_uncertainty = struct('mu',0,'Sigma',0);
    measurement_uncertainty = struct('mu',0,'Sigma',0);
    
    SystemStruct = struct('A',A,'B',B,'B_d',B_disturbance,'Delta',Delta,...
        'C',C,'measurement_uncertainty',measurement_uncertainty,...
        'prediction_uncertainty',prediction_uncertainty,...
        'model_uncertainty',model_uncertainty);
end