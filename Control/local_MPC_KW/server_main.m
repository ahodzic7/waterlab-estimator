clear all; clc; 

%%
controlType = 2;                                                                         % switch between on/off and MPC

%% ============================================ Control setup ======================================
MPC_init_KW                                                                           % Load when comparing lab results

%% ===================================  Build dynamics & optimization  =============================
                                                                    % build simulator dynamics
if controlType == 2
    MPC_builder_PDE;
end

%% Communication setup 

%%
number_of_receiving_data = 7;
number_of_sending_data = 6;

% Holding is writting registers (size is number of variables x2)
% 6 states + current time = 7
DataBaseHolding = uint16(zeros(1,number_of_receiving_data*4));
% Input is readin registers
% 2 pump references + 2 overflow slacks + 2 tank references + 2 slack tightenings = 8
DataBaseInput = uint16(zeros(1,number_of_sending_data*4));
% Ignore Coils
DataBaseCoils = logical(0);

Client_IP = 'localhost';
Port_Number = 502;

display('Server is running!')

while(1)
    ModBusTCP = openConnectionServer(Client_IP, Port_Number)
    %Modbus server
    while ~ModBusTCP.BytesAvailable
        %wait for the response to be in the buffer
    end
    
    %Save old measurements
    oldDataBaseHolding = DataBaseHolding;
    
    %Handle new request
    [DataBaseInput,DataBaseHolding] = handleRequest(ModBusTCP, ...
                              DataBaseInput,DataBaseHolding,DataBaseCoils);
   
    %MPC
    if(any(oldDataBaseHolding ~= DataBaseHolding))
        Updated_Measurements_data = unit16Be2doubleLe(DataBaseHolding);
      
        %Run MPC
        X0 = Updated_Measurements_data(1,1:6)'/100;
        time = Updated_Measurements_data(1,7);
        
        time = time +1;
        
        if controlType == 1
        onoff_control;
        elseif controlType == 2    
        [U_MPC,S_MPC,Y_MPC,lam_g,x_init] = OCP(X0, D_sim(:,(time)*(t_step)-(t_step-1):t_step:(time-1)*t_step + (Hp)*t_step-(t_step-1)),...
            P_sim, X_ref_sim(:,(time)*(t_step)-(t_step-2):t_step:(time-1)*t_step + (Hp)*t_step-(t_step-2)),...
            lam_g, x_init, dt_sim);

        U_opt = full(U_MPC);
        S_opt = full(S_MPC);
        end
        
        output = [U_opt; S_opt; X_ref_sim(:,time)];
        
%         U = output(1:2,:);
%         Overflow = output(3:4,:);
%         X_ref = output(5:6,:);
%         S_ub = output(7:8,:);
        
        %Prepare calculations for sending to client
        data2Send = flip(output');
        DataBaseInput = flip(typecast(data2Send,'uint16'));
    end
    
    fclose(ModBusTCP);
end 

