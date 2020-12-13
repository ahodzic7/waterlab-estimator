close all;
clear
clc
%% Calculate one step residuals
% Setup Model
NumberOfStates = 5;
% For lab
p = [0.0348661973172381,0.0548687811513346,0.000100000000000000,...
        -0.00169231647536930,0.0342193052314381,0.203718327157626];
phi = [1,1/4.908738521234052];
DeltaT = 0.5;

% For Basic Test
% p = [0.00573303726337542,0.00455712815840905,0.00100679057458217,...
%     -6.99861868943381e-05,0.00307844320832101,0.00500000000000000];
% phi = [1,0.00500000000000000];
% DeltaT = 60;

A = BuildA(NumberOfStates,p,phi,DeltaT);
B = BuildB(NumberOfStates,p,phi,DeltaT);
Delta = BuildDelta(NumberOfStates, p,DeltaT);

measurements = [h; T2];
%% Free running
x_est(1,:) = sys_final.Report.Parameters.X0';

for k = 1:1:size(output,1)-1
    x_est(k+1,:) = (A*x_est(k,:)' + B*input(k,:)' + Delta)';
end
t = 1:1:size(output,1);
figure
plot(t,measurements(1,1:end),'b')
hold on
plot(t,x_est(1:end,1)','r')

%% 1 step prediction

x_est(1,:) = [0 0 0 0 0]';
for k = 1:1:size(output,1)-1
    x_est(k+1,:) = (A*measurements(:,k) + B*input(k,:)' + Delta)';
end

t = 1:1:size(output,1);
figure
plot(t,measurements(1,1:end),'b')
hold on
plot(t,x_est(1:end,1)','r')
%% Calculate Residuals
residual= (output-x_est);

%Plot
figure
for index = 1:1:Nx
    ax(index) = subplot(size(output,2),1,index);
    plot(0:dataTimeStep:(size(output(:,1),1)-1)*dataTimeStep,residual(:,index)','b','LineWidth',1);
    hold on;
    yline(0,'k','LineWidth',1.5);
    if index == Nx
        ylabel(['$r_{T2}$'],'interpreter','latex');
    else
        ylabel(['$r$' num2str(index)],'interpreter','latex');
    end
    if index == 1
        title('Residuals','interpreter','latex')
    end
    grid on;
end
%% Find residual distribution
%Assuming normal independant residauls
figure
state_muHat = [];
state_sigmaHat = [];
for i = 1:1:Nx
    %Approximate distribution
    [muHat,sigmaHat] = normfit(residual(:,i));
    state_muHat = [state_muHat, muHat];
    state_sigmaHat = [state_sigmaHat, sigmaHat];
    t = -0.08:0.0001:0.08;
    pdf = normpdf(t,muHat,sigmaHat)/100;
    
    %Plot
    hold on;
    subplot(Nx,1,i);
    if i == Nx
        histogram(residual(:,i),200,'Normalization','probability');
        xlabel(['$r_{T2} [dm]$'],'interpreter','latex');
        axis([-0.05 0.05 0 0.9])
    else
        histogram(residual(:,i),120,'Normalization','probability');
        xlabel(['$r$' num2str(i) ' $[dm]$' ],'interpreter','latex');
        axis([-0.025 0.025 0 0.25])
    end
    ylabel(['$P[R$' num2str(i) '$= r$' num2str(i) '$_{k}]$'],'interpreter','latex');
    grid on;
    %plot(t,pdf);
end

state_muHat
state_sigmaHat

%% Find residual distribution for model noise 1try
figure
state_muHat = [];
state_sigmaHat = [];
 Nx = Nx;
for i = 1:1:Nx
    if i == Nx
        model_residual = residual(:,i);
    else
        residual_col = residual(:,i)
        model_residual = residual_col(residual_col < 0.005)
        model_residual = model_residual(model_residual > -0.005)
    end
    
    %Approximate distribution
    [muHat,sigmaHat] = normfit(model_residual);
    state_muHat = [state_muHat, muHat];
    state_sigmaHat = [state_sigmaHat, sigmaHat];
    t = -0.08:0.0001:0.08;
    pdf = normpdf(t,muHat,sigmaHat)/100;
    
    %Plot 
    subplot(Nx,1,i);
    hold on;
    if i == Nx
        histogram(model_residual,200,'Normalization','probability');
        xlabel(['$r_{T2} [dm]$'],'interpreter','latex');
        axis([-0.05 0.05 0 0.9])
    else
        histogram(model_residual,10,'Normalization','probability');
        xlabel(['$r$' num2str(i) ' $[dm]$' ],'interpreter','latex');
        axis([-0.025 0.025 0 0.25])
    end
    ylabel(['$P[R$' num2str(i) '$= r$' num2str(i) '$_{k}]$'],'interpreter','latex');
    grid on;
    %plot(t,pdf);
end
state_muHat
state_sigmaHat

%% Find residual distribution for measurement noise 2 try
figure
noise_muHat = [];
noise_sigmaHat = [];
 Nx = Nx;
for i = 1:1:Nx-1

    residual_col = residual(:,i)
    noise_residual = residual_col(residual_col < 0.005)
    noise_residual = noise_residual(noise_residual > -0.005)
    noise_residual(noise_residual ~= 0) = 0

    for t = 0:1:9
        temp_residual_high = residual_col(residual_col > 0.005 + 0.01 * t & residual_col < 0.015 + 0.01 * t) - 0.01 * ( t+1) ;
        temp_residual_high(temp_residual_high ~= (t + 1)) = 0.01 *(t + 1);

        temp_residual_low=  residual_col(residual_col < - 0.005 - 0.01 * t & residual_col > -0.015 - 0.01 *t) + 0.01 * ( t+1);
        temp_residual_low( temp_residual_low ~= -(t + 1) ) = -0.01 *(t + 1);
        noise_residual = [noise_residual; temp_residual_high; temp_residual_low];

    end
    
    
    %Approximate distribution
    [muHat,sigmaHat] = normfit(noise_residual);
    noise_muHat = [noise_muHat, muHat];
    noise_sigmaHat = [noise_sigmaHat, sigmaHat];
    t = -0.08:0.0001:0.08;
    pdf = normpdf(t,muHat,sigmaHat)/100;
    
    %Plot
    subplot(Nx-1,1,i);
    plot(t,pdf);
    hold on;
    histogram(noise_residual,200,'Normalization','probability');
    xlabel(['$v$' num2str(i) '$_k [dm]$' ],'interpreter','latex');
    axis([-0.025 0.025 0 0.8])
    ylabel(['$P[V$' num2str(i) '$= v$' num2str(i) '$_{k}]$'],'interpreter','latex');
    grid on;
end
noise_muHat 
noise_sigmaHat 

%% Find residual distribution for model noise 2 try
figure
model_muHat = [];
model_sigmaHat = [];
 Nx = Nx;
for i = 1:1:Nx
    if i == Nx
        model_residual = residual(:,i);
    else
        residual_col = residual(:,i)
        model_residual = residual_col(residual_col < 0.005)
        model_residual = model_residual(model_residual > -0.005)
        
        for t = 0:1:9
            model_residual= [model_residual; residual_col(residual_col > 0.005 + 0.01 * t & residual_col < 0.015 + 0.01 * t) - 0.01 * ( t+1)] ;
            model_residual= [model_residual; residual_col(residual_col < - 0.005 - 0.01 * t & residual_col > -0.015 - 0.01 * t) + 0.01 * ( t+1)] ;
        end
    end
    
    %Approximate distribution
    [muHat,sigmaHat] = normfit(model_residual);
    model_muHat = [model_muHat, muHat];     
    model_sigmaHat = [model_sigmaHat, sigmaHat];
    t = -0.08:0.0001:0.08;
    
    if i == Nx
    pdf = normpdf(t,muHat,sigmaHat)/10;
    else
    pdf = normpdf(t,muHat,sigmaHat)/1000;
    end
        
    %Plot
    subplot(Nx,1,i);
    plot(t,pdf);
    hold on;
    if i == Nx
        histogram(model_residual,200,'Normalization','probability');
        xlabel(['$w_{T2} [dm]$'],'interpreter','latex');
        axis([-0.08 0.08 0 1.5])
    else
        histogram(model_residual,10,'Normalization','probability');
        xlabel(['$w$' num2str(i) ' $_k[dm]$' ],'interpreter','latex');
        axis([-0.004 0.004 0 1])
    end
    ylabel(['$P[W$' num2str(i) '$= w$' num2str(i) '$_{k}]$'],'interpreter','latex');
    grid on;
end
model_muHat 
model_sigmaHat