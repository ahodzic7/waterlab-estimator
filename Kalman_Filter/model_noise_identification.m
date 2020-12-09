
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

for index = 1:1:size(output,1)-1
    x_est(index+1,:) = (A*measurements(:,index) + B*input(index,:)' + Delta)';
end
residual = (output-x_est);

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
% Nx = Nx-1;
for i = 1:1:Nx
    %Approximate distribution
    [muHat,sigmaHat] = normfit(residual(:,i));
    state_muHat = [state_muHat, muHat];
    state_sigmaHat = [state_sigmaHat, sigmaHat];
    t = -0.08:0.0001:0.08;
    pdf = normpdf(t,muHat,sigmaHat)/100;
    
    %Plot
    subplot(Nx,1,i);
    histogram(residual(:,i),30,'Normalization','probability');
    hold on;
    plot(t,pdf);
    if i == Nx
        ylabel(['$|r_{T2}|$'],'interpreter','latex');
    else
        ylabel(['$|r$' num2str(i) '$|$' ],'interpreter','latex');
    end
end

state_muHat
state_sigmaHat