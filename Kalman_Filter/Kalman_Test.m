%% ================================================ Run Kalman =============================================
EstimatedX = [h(1:end,1); T2(1)];
ErrorCo = eye(5)*1;            %Large number
ControlInput = [Q(1,1), Q(2,1)];
X = zeros(size(EstimatedX,1),size(h,2));

for time = 1:1:size(h,2)
    sysMeas =[h(2,time);h(4,time); T2(time)];
    ControlInput = [Q(1,time); Q(2,time)];
    [EstimatedX,ErrorCo,KalmanGain] = KalmanFilterForLab(sysMeas,EstimatedX, ErrorCo, ControlInput);
    X(:,time) = EstimatedX;
end

ErrorCo
KalmanGain


figure(1)
subplot(5,1,1)
plot(h(1,:));
hold on
plot(X(1,:));

subplot(5,1,2)
plot(h(2,:));
hold on
plot(X(2,:));

subplot(5,1,3)
plot(h(3,:));
hold on
plot(X(3,:));

subplot(5,1,4)
plot(h(4,:));
hold on
plot(X(4,:));

subplot(5,1,5)
plot(T2);
hold on
plot(X(5,:));