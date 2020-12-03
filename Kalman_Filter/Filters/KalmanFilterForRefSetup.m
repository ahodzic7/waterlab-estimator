function  [EstimatedX,ErrorCo,KalmanGain] = KalmanFilterForRefSetup(SystemMeas,PrevEstimatedX,PrevErrorCo,ControlInput)
%KALMANFILTERFORREFSETUP is a Kalman filter for the reference simulation setup

%System Parameters
    DeltaT = 0.5;
    p = [0.0057    0.0046    0.0010   -0.0001    0.0031    0.0050];
    phi = [1,1/200];
    variance = [0.0168    0.0166    0.0145    0.0191    0.0538]'.^2
    Qm = diag(variance);      %Covariance of model noise
    Rm = eye(3)*0.001;          %Covariance of messurment noise
    
%System Model
NumberOfStates = 5;
A = BuildA(NumberOfStates,p,phi,DeltaT);
B = BuildB(NumberOfStates,p,phi,DeltaT);
Delta = BuildDelta(NumberOfStates,p,DeltaT);
C = zeros(NumberOfStates-2,NumberOfStates);
C(1,1) = 1;
C(2,3) = 1;
C(3,5) = 1;

%Prediction Step
    PredictedX = A * PrevEstimatedX + B * ControlInput + Delta;
    PredictedP = A * PrevErrorCo * A' + Qm;
    
%Compute KalmanGain
    KalmanGain = PredictedP * C' /(C * PredictedP * C' + Rm);

%Compute EstimatedX
    EstimatedX = PredictedX + KalmanGain * (SystemMeas - C * PredictedX);

%Compute ErrorCo
    ErrorCo = PredictedP - KalmanGain * C * PredictedP;
end

