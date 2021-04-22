function [EstimatedX] = Kalmanfilter(ControlInput,SystemMeas)
persistent PrevEstimatedX;
persistent PrevErrorCo;
persistent A B Bd Delta
dT = 0.5;    
C = eye(6);
model_uncert = 0;
controller_uncert = 0;
meassurement_uncert = 0;

if is empty(A)
    %Setup system matrices
    sys = evalin('base','sys');
    A = full(sys.A(dT));
    B = full(sys.B(dT));
    Bd = full(sys.Bd(dT));
    Delta = full(sys.Delta(dT));
end

Qm = model_uncert + B * controller_uncert * B';
Rm = meassurement_uncert;

%Prediction Step
    PredictedX = A * PrevEstimatedX + B * ControlInput...
        + Bd * Disturbance + Delta;
    PredictedP = A * PrevErrorCo * A' + Qm;
    
%Compute KalmanGain
    KalmanGain = PredictedP * C' /(C * PredictedP * C' + Rm);

%Compute EstimatedX
    EstimatedX = PredictedX + KalmanGain * (SystemMeas - C * PredictedX);
    PrevEstimatedX = EstimatedX;
    
%Compute ErrorCo
    ErrorCo = PredictedP - KalmanGain * C * PredictedP;
    PrevErrorCo = ErrorCo;

