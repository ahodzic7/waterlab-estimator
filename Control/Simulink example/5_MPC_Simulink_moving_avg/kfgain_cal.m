function [K,Pc,Pp]=kfgain_cal(Phi,Cd,Pp,Q,R,n_states)

K=Pp*Cd'/(Cd*Pp*Cd'+R);     %Kalman Gain
Pc = (eye(n_states)-K*Cd)*Pp;      %Auto covariance of error of corrected estimate
Pp=Phi*Pc*Phi'+Q;           %Auto covariance of error of predicted estimate of next time step

end
