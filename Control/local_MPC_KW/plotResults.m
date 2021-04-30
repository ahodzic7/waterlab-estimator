
dt_MPC = dt_original*t_resample/data_timeUnit;

plotEnabler = 1;
if plotEnabler == 1
figure
ax(1) = subplot(3,2,1);
plot(D_sim(1,1:t_resample:N*t_resample)')
% hold on
% plot(d(1,:))
leg = legend('$d_{1}$');
set(leg,'Interpreter','latex');
title('Disturbances','interpreter','latex')
%
flow_model_est = P_sim(2)*(((X_sim(Nxt+ Nxp,:)).^(5/3)) ./ ((X_sim(Nxt+ Nxp,:) + P_sim(3)).^(2/3)));
ax(2) = subplot(3,2,2);
plot(flow_model_est)%+ test(1,1:end-1))
% hold on
% plot(d(3,:) + u(1,:))
leg = legend('$d_{2}$');
set(leg,'Interpreter','latex');
title('Disturbances','interpreter','latex')
%
ax(3) = subplot(3,2,3);
plot(X_sim(1,:)')
hold on
plot(X_ref_sim(1,1:t_resample:N*t_resample),'black')
hold on
%plot((Kt/dt_MPC)*S_opt(1,:)','red')
hold on
plot(max_t1*ones(N,1),'red--')
hold on
plot(min_t1*ones(N,1),'red--')
% hold on
% plot(x(1,:),'red')
leg = legend('$x_{t1}$','Reference','Overflow');
set(leg,'Interpreter','latex');
title('Tank t1 state','interpreter','latex')
%
ax(4) = subplot(3,2,4);
plot(X_sim(2,:)')
hold on
plot(X_ref_sim(2,1:t_resample:N*t_resample),'black')
hold on
%plot((Kt/dt_MPC)*S_opt(2,:)','red')
hold on
plot(max_t2*ones(N,1),'red--')
hold on
plot(min_t2*ones(N,1),'red--')
% hold on
% plot(x(2,:),'red')
leg = legend('$x_{t2}$','Reference','Overflow');
set(leg,'Interpreter','latex');
title('Tank t2 state','interpreter','latex')
%
ax(5) = subplot(3,2,5);
plot(U_opt(1,:)')
hold on
plot(u1_on*ones(N,1),'red--')
hold on
plot(u1_off*ones(N,1),'red--')
% hold on
% plot(u(1,:),'red')
leg = legend('$u_{t1}$','Limit');
set(leg,'Interpreter','latex');
title('Pump 1','interpreter','latex')
%
ax(6) = subplot(3,2,6);
plot(U_opt(2,:)')
hold on
plot(u2_on*ones(N,1),'red--')
hold on
plot(u2_off*ones(N,1),'red--')
% hold on
% plot(u(2,:),'red')
leg = legend('$u_{t2}$','Limit');
set(leg,'Interpreter','latex');
title('Pump 2','interpreter','latex')

linkaxes(ax, 'x')

figure
title('Pipe states','interpreter','latex')
for i = 1:Nxp
    subplot(Nxp,1,i)
    plot(X_sim(Nxt+i,:)');
    title(['Pipe state', num2str(i)],'interpreter','latex')
end
end