%% LevelControl.m
% """
% Level control. Uses On/Off fixed-speed pump configuration with respect 
% to water level measurements. Flag variables account for filling/emptying switching feature.
%
% Input:  tank level 
%         flag
%         Qon, Qoff
%
% Output: flow Qon/Qoff setpoints
% """

try
    %ctrl_1 = 0;
    %if ctrl_1 == 1
    % On/Off level control for tank 1
    if X_sim(1,i) >= max_t1 
        input1 = u1_on;
    elseif X_sim(1,i) <= min_t1 
        input1 = u1_off; 
    end 
    U_opt(1,i) = input1;
    %end
    % On/Off level control for tank 2
    if X_sim(2,i) >= max_t2 
        input2 = u2_on;
    elseif X_sim(2,i) <= min_t2 
        input2 = u2_off;
    end 
    U_opt(2,i) = input2;
catch
    fprintf('Error in on/off control \n');
end