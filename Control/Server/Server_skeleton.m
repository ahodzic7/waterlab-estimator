%% ======================== Initialize the MPC ============================
clear all;
clc;
addpath(genpath('Lab_Stochastic_MPC_full_system_Linear_DW'));
SMPC_init_DW


%%
% Holding is writting registers (size is number of variables x2)
% 6 states + current time = 7
DataBaseHolding = uint16(zeros(1,7*4));
% Input is readin registers
% 2 pump references + 2 overflow slacks + 2 tank references + 2 slack tightenings = 8
DataBaseInput = uint16(zeros(1,8*4));
DataBaseCoils = logical(0)
ModBusTCP = openConnectionServer('192.168.100.123', 502)

while(1)
    %Modbus server
    while ~ModBusTCP.BytesAvailable
        %wait for the response to be in the buffer
    end
    oldDataBaseHolding = DataBaseHolding;
    [DataBaseInput,DataBaseHolding] = handleRequest(ModBusTCP, ...
                              DataBaseInput,DataBaseHolding,DataBaseCoils);
   
    %MPC
    DataBaseHolding
    if(sum(oldDataBaseHolding ~= DataBaseHolding))
        Updated_Measurements_data = unit16Be2doubleLe(DataBaseHolding);
      
        %Run MPC
        X0 = Updated_Measurements_data(1,1:6);
        time = Updated_Measurements_data(1,7);
%         output = SMPC_full_DW(X0, time); 
%         U = output(1:2,:);
%         Overflow = output(3:4,:);
%         X_ref = output(5:6,:);
%         S_ub = output(7:8,:);
    end
end   