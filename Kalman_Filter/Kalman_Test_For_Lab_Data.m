
%% ================================================ Prepare Data =============================================
N_sensors = 4;                                                             % Select section number, i.e. pick number of level sensor data
h(1:N_sensors,:) = uConv(data(3:1:3+N_sensors-1,startDataIndex:endDataIndex), ["mmTodm"]);
Q(1,:) = uConv(data(9,startDataIndex:endDataIndex), ["1/minTo1/s", ""]); % Select in/outflows
Q(2,:) = uConv(data(10,startDataIndex:endDataIndex), ["1/minTo1/s", ""]);
T2 = uConv(data(8,startDataIndex:endDataIndex), ["mmTodm"]);                       % Select tanks

%% ================================================ Run Kalman =============================================
EstimatedX = [h(1:end,1); T2(1)];
ErrorCo = 1;
ControlInput = [Q(1,1), Q(2,1)];
X = zeros(size(EstimatedX,1),size(h,2));

for time = 1:1:size(h,2)
    sysMeas =[h(1,time);h(3:end,time); T2(time)];
    ControlInput = [Q(1,time); Q(2,time)];
    [EstimatedX,ErrorCo,KalmanGain] = KalmanFilterForLab(sysMeas,EstimatedX, ErrorCo, ControlInput);
    X(:,time) = EstimatedX;
end

figure(1)
subplot(4,1,1)
plot(h(1,:));
hold on
plot(X(1,:));

subplot(4,1,2)
plot(h(2,:));
hold on
plot(X(2,:));

subplot(4,1,3)
plot(h(3,:));
hold on
plot(X(3,:));

subplot(4,1,4)
plot(h(4,:));
hold on
plot(X(4,:));

function [EstimatedX,ErrorCo,KalmanGain] = KalmanFilterForLab(SystemMeas,PrevEstimatedX,PrevErrorCo,ControlInput)
%System Parameters
    DeltaT = 0.5;
    p = [0.0057    0.0046    0.0010   -0.0001    0.0031    0.0050];
    phi = [1,1/200];
    Qm = eye(5)*0.003;      %Covariance of model noise
    Rm = eye(4)*0;          %Covariance of messurment noise
    
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
    PredictedP = A * PrevErrorCo * A' + Qm;
    
%Compute KalmanGain
    KalmanGain = PredictedP * C' /(C * PredictedP * C' + Rm);

%Compute EstimatedX
    EstimatedX = PredictedX + KalmanGain * (SystemMeas - C * PredictedX);

%Compute ErrorCo
    ErrorCo = PredictedP - KalmanGain * C * PredictedP;
end