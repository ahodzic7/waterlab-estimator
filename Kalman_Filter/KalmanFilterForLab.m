function [EstimatedX,ErrorCo,KalmanGain] = KalmanFilterForLab(SystemMeas,PrevEstimatedX,PrevErrorCo,ControlInput)
%KALMANFILTERFORLAB Summary of this function goes here
%   Detailed explanation goes here

function [EstimatedX,ErrorCo,KalmanGain] = KalmanFilterForLab(SystemMeas,PrevEstimatedX,PrevErrorCo,ControlInput)
%System Parameters
    DeltaT = 0.5;
    p = [0.0368    0.0546   -0.0069   -0.0021    0.0366];
    phi = [1,0.2037];
    Q = eye(5)*1;
    R = eye(4)*1;
    
%System Model
NumberOfStates = 5;
A = BuildA(NumberOfStates,p,phi,DeltaT);
B = BuildB(NumberOfStates,p,phi,DeltaT);
C = zeros(NumberOfStates-1,NumberOfStates);
C(1,1) = 1;
C(2,3) = 1;
C(3,4) = 1;
C(4,5) = 1;

%Prediction Step
    PredictedX = A * PrevEstimatedX + B * ControlInput;
    PredictedP = A * PrevErrorCo * A' + Q;
    
%Compute KalmanGain
    KalmanGain = PredictedP * C' /(C * PredictedP * C' + R);

%Compute EstimatedX
    EstimatedX = PredictedX + KalmanGain * (SystemMeas - C * PredictedX);

%Compute ErrorCo
    ErrorCo = PredictedP - KalmanGain * C * PredictedP;
end
end

