clear all;
clc
%% System paramters:
NumberOfStates = 10;
parameters = [0.0578290979772847,0.137832091474361,0.000100000000000000,-0.00513392718034462,0.100000000000000];
phi = [1/4.908738521234052,1/4.908738521234052];
DeltaT = 6;

%%Create Model:
% System matrices - x(n+1) = A x(n) + B u(n) + Bd ud(n) + Delta
A = BuildA(NumberOfStates,parameters,phi,DeltaT);
B = BuildB(NumberOfStates,parameters,phi,DeltaT);
Bd = BuildBd(NumberOfStates,2,parameters,phi,DeltaT);
Delta = BuildDelta(NumberOfStates, parameters,DeltaT);

%%Set up free running model
N = 200;
u = [5/60;0];
X = zeros(NumberOfStates,N);
X(:,1) = [6.5, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0];
%%Run
for n = 1:1:N
    X(:,n+1) = A * X(:,n) + B * u + Delta;
end

%Plot
figure
for i = 1:1:NumberOfStates
subplot(NumberOfStates,1,i);
plot(X(i,:));
hold on;
end
