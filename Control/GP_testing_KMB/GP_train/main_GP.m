clear all;
clc
%% Setup system 

% load('.\data\x_val')
% load('.\data\u_val')
% load('.\data\d_val')
% x = x_val;
% u = u_val;
% d = d_val;

load('.\data\x')
load('.\data\u')
load('.\data\d')
load('.\data\Kt')

t_resample = 20;
dt_original = 0.5;
data_timeUnit = 60;                                                             % flow is in [60s]
dt = dt_original*t_resample/60;                                                 % 10 [min]
                                                                                    
Nxt = 2;                                                                        % tank state size
Nxp = 4;                                                                        % pipe states size
Nx = Nxt + Nxp;                                                                 % full state size

% Nominal dynamics
At = (eye(Nxt)); 
Bt = -diag([dt/Kt, dt/Kt]); 
Et = diag([dt/Kt dt/Kt]);

A = [At, zeros(Nxt,Nxp); zeros(Nxp,Nx)];
B = [Bt; zeros(Nxp,Nxt)];
E = [Et, zeros(Nxt,1); zeros(Nxp,3)];

f = [At*x(1:Nxt,1:end-1) + Bt*u(1:Nxt,1:end-1) + Et*d(1:Nxt,1:end-1);...
    zeros(1,length(x(1:Nxt,1:end-1)));...
    zeros(1,length(x(1:Nxt,1:end-1)));...
    zeros(1,length(x(1:Nxt,1:end-1)));...
    zeros(1,length(x(1:Nxt,1:end-1)))];

% % Sanity check - tank dynamics with known disturbance
xt_check = At(1)*x(1,:) + Bt(1)*u(1,:) + Et(1)*d(1,:);

%%
% Unknown dynamics
% g(x2,x3,x4,x5,u1,u3,d2)                                                           

Bd = eye(6);
%Bd = [zeros(1,5); eye(5)];                                                      % mapping matrix

y_unfiltered = pinv(Bd) * (x(:,2:end) - f);                                      % residuals (output set)
z = [x; u; d];                                                               % mm to dm conversion
%z = [x(2:Nx,:); u; d(3,:)];                                                     % state-input-disturbance pairs (input set)

% remove outliers
for i = 1:Nx
    y(i,:) = medfilt1(y_unfiltered(i,:),3);                                     % 
end

n = 500;%350;                                                                        % training set length

% Add noise to tank residuals
y(1:Nxt,:) = y(1:Nxt,:) + 0.01*randn(Nxt,size(y,2)); 
% Add noise to pipe residuals
y(Nxt+1:Nx,:) = y(Nxt+1:Nx,:) + 0.005*randn(Nxp,size(y,2)); 

% Sanity check - residuals 
figure                                                                          % tank states
for i = 1:Nxt
    plot(y(i,:))
    hold on
end
plot(zeros(size(y,2),1),'black--')
title('Residuals for tanks','interpreter','latex')
leg = legend('$[y]_{1}$','$[y]_{2}$','zero');
set(leg,'Interpreter','latex');
xlabel('Time [10 s]','interpreter','latex')
ylabel('Level [dm]','interpreter','latex')

figure                                                                          % pipe states
for i = Nxt+1:Nxt+Nxp
    plot(y(i,:))
    hold on
end
plot(zeros(size(y,2),1),'black--')
title('Residuals for pipe states','interpreter','latex')
leg = legend('$[y]_{3}$','$[y]_{4}$','$[y]_{5}$','$[y]_{6}$','zero');
set(leg,'Interpreter','latex');
xlabel('Time [10 s]','interpreter','latex')
ylabel('Level [$dm$]','interpreter','latex')

%% Train GPs
gps=cell(Nx,1);

% Initialize signal variance
sigma0 = std(y');

tic 
for i=1:Nx
    gps{i} = fitrgp(z(:,1:n)',y(i,1:n)','OptimizeHyperparameters','auto',...
        'KernelFunction','squaredexponential','HyperparameterOptimizationOptions',...
        struct('UseParallel',true,'MaxObjectiveEvaluations',30),'Sigma',sigma0(i),'Standardize',1);
    % kernels: 'matern32' 'squaredexponential'
end
toc

%% load
%save('.\GPs\gps')
%load('.\GPs\gps')

%% 
for i = 1:Nx
num_gp = i;                                                                     % gp selection
gp1=gps{num_gp};
np = size(x,2)-n-1;                                                             % number of predictions

figure
subplot(2,1,1)
respred1 = resubPredict(gp1);
plot(y(num_gp,1:n));
hold on 
plot(respred1)
title('1 step prediction - training data','interpreter','latex')
leg = legend('Data','Model');
set(leg,'Interpreter','latex');
ylabel('Level [$dm$]','interpreter','latex')
% 
subplot(2,1,2)
[respred1,~,ress_ci] = predict(gp1, z(:,n:(n+np))');
plot(y(num_gp,n:(n+np)));
hold on 
plot(respred1)
hold on
ciplot(ress_ci(:,1),ress_ci(:,2)) 
title('1 step prediction - validation data','interpreter','latex')
leg = legend('Data','Model');
set(leg,'Interpreter','latex');
xlabel('Time [10 s]','interpreter','latex')
ylabel('Level [$dm$]','interpreter','latex')
end


%% Np long prediction 

x0 = x(:,n);
x_hat = zeros(size(x,1),np);
x_hat(:,1) = x0;
%ress_ci_up = zeros(size(x,1),np);
%ress_ci_down = zeros(size(x,1),np);
for i = 1:np
    ress=zeros(Nx-1,1);
    %ress_ci=zeros(Nx,2);

    for j = 1:Nx
       [ress(j),~,ress_ci_np(j,:)] = predict(gps{j}, ([x_hat(:,i);u(:,n+i-1);d(:,n+i-1)])');
    end
    if sum(isnan(ress)) > 0
        return
    end
    x_hat(:,i+1)= [At*x_hat(1:Nxt,i) + Bt*u(1:Nxt,n+i-1) + Et*d(1:Nxt,n+i-1); 0; 0; 0; 0] + ress;
end
%% Plot NP long prediction

figure                                                                          % tank states
for i = 1:Nxt
subplot(Nxt,1,i)
plot(x(i,n:(n+np)),'.-');
hold on
plot(x_hat(i,:))
title(['Tank',' - ','x', num2str(i)],'interpreter','latex')
leg = legend('Data','Model');
set(leg,'Interpreter','latex');
ylabel('Level [$dm$]','interpreter','latex')
end
xlabel('Time [10s]','interpreter','latex');

figure                                                                          % pipe states
for i = Nxt+1:Nxt+Nxp
subplot(Nxp,1,i-Nxt)
plot(x(i,n:(n+np)),'.-');
hold on
plot(x_hat(i,:))
title(['Pipe',' - ','x', num2str(i)],'interpreter','latex')
leg = legend('Data','Model');
set(leg,'Interpreter','latex');
ylabel('Level [$dm$]','interpreter','latex')
end
xlabel('Time [10s]','interpreter','latex');

%% Post-process

tic
% Build sigma_f vector 
sigma_f = zeros(Nx,1);
sigma_L = zeros(Nx,1);

for i = 1:Nx 
    sigma_f(i) = gps{i}.KernelInformation.KernelParameters(2);
    sigma_L(i) = gps{i}.KernelInformation.KernelParameters(1);
end

% Build L diagonal matrices
L=cell(1,Nx);
for i = 1:Nx
    L_scale_vector = sigma_L(i)*ones(size(z,1),1);
    L{i} = diag(L_scale_vector);
end

% Build sigma vector
for i = 1:Nx
sigma(i) = gps{i}.Sigma;
end

%% Build training dataset and Beta offsets

 z_train = z(:,1:n);
 y_train = y(:,1:n);

for i = 1:Nx
   Beta(i,:) = gps{i}.Beta;
end


%%

save('.\GP_parameters','sigma_f','sigma_L','sigma','z_train','y_train','Beta')


 %% K matrices
% K = cell(1,size(y,1));
% M = size(z(1:n),2);
% N = 1;
% kernel = zeros(M+N,M+N);
% for k = 1:size(y,1)
%     kernel = (sigma_f(k)^2) * exp(-0.5*(squareform(pdist(z(:,1:M+N)')).^2)/(L{k}(1)^2));
%     K{k} = kernel;
% end
% 
% % x - training state-input pairs
% % z - new state-input pair 
% 
% for i = 1:Nx
%     K_xx{i} = K{i}(1:M,1:M);
%     K_zz{i} = K{i}(M+N,M+N);
%     K_xz{i} = K{i}(1:M,M+N);
%     K_zx{i} = K_xz{i}';
% end
% 
% %% Mean and covariance 
% for i = 1:Nx
%     mu_d(i) = K_zx{i}/(K_xx{i} + eye(M)*gps{i}.Sigma^2) * y(i,1:M)';
%     sigma_d(i) = K_zz{i} - K_zx{i}/(K_xx{i} + eye(M)*gps{i}.Sigma^2) * K_xz{i};
% end
% 
% %% Mean function gradient 
% grad_mu = zeros(Nx,Nx);
% alpha = cell(Nx,1);
% 
% for i = 1:Nx
%     alpha{i} = (K_xx{i} + eye(M)*gps{i}.Sigma^2)\y(i,1:M)';                        % N x 1
%     grad_mu(i,:) = ((-1/L{i}(1))*(z(1:Nx,M+N) - z(1:Nx,1:M))*(K_zx{i}'.*alpha{i}))';  
% end

%% Open-loop  









