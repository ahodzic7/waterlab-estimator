
dt = 1;

% Tank states
x_plot{1}.XData=(0:i)*dt;
xpl{2}.XData=(0:i)*dt;
x_plot{1}.YData=X_sim(1,:);
xpl{2}.YData=X_sim(2,:);

x_plot{1}.Parent.XLim=[0 i+Hp];
xpl{2}.Parent.XLim=[0 i+Hp];
hold(x_plot{1}.Parent,'on')

hold(x_plot{1}.Parent,'off')
hold(xpl{2}.Parent,'on')


%inputs
upl{1}.XData=(0:i-1)*dt;
upl{2}.XData=(0:i-1)*dt;
upl{1}.YData=U_opt(1,:);
upl{2}.YData=U_opt(2,:);

upl{1}.Parent.XLim=[0 i+Hp];
upl{2}.Parent.XLim=[0 i+Hp];

xlim([0,i+Hp])