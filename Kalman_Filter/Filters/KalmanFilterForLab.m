function [EstimatedX,ErrorCo,KalmanGain] = KalmanFilterForLab(SystemMeas,PrevEstimatedX,PrevErrorCo,ControlInput)
%KALMANFILTERFORLAB is a Kalman filter for the AAU Smart Water Lab setup

%System Parameters
    DeltaT = 0.5;
    p = [0.0367815377915250,0.0546308635442870,-0.00685599772690400,-0.00209121684418000,0.0365705383994410,0.203718327157626];
    phi = [1,1/4.908738521234052];
    variance = [0.0195    0.0156    0.0134    0.0183    0.4387]'.^2
    Qm = diag(variance);       %Covariance of model noise
    Rm = eye(3)*0.01;          %Covariance of messurment noise
    
%System Model
NumberOfStates = 5;
A = BuildA(NumberOfStates,p,phi,DeltaT);
B = BuildB(NumberOfStates,p,phi,DeltaT);
Delta = BuildDelta(NumberOfStates,p,DeltaT);
C = zeros(NumberOfStates-2,NumberOfStates);
C(1,2) = 1;
C(2,4) = 1;
C(3,5) = 1;

%Prediction Step
    PredictedX = A * PrevEstimatedX + B * ControlInput  + Delta;
    PredictedP = A * PrevErrorCo * A' + Qm;
    
%Compute KalmanGain
    KalmanGain = PredictedP * C' /(C * PredictedP * C' + Rm);

%Compute EstimatedX
    EstimatedX = PredictedX + KalmanGain * (SystemMeas - C * PredictedX);

%Compute ErrorCo
    ErrorCo = PredictedP - KalmanGain * C * PredictedP;
end

