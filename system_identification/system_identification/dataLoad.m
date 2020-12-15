% Loading the data into a .mat file
% 
% The data must be in the following form:
% 
% """
% 1     : time steps 
% 2:5   : pipe levels
% 6     : NAN (tank2 inflow) 
% 7     : tank2 level
% 8     : Gravity pipe inflow (input) 
% 9     : tank2 outflow 
% 10    : tank2 area 
% 11    : real time 
% 12    : lateral inflow (disturbance)
% 13    : inflow to tank1 
% 14:17 : [pump1_ref(inflow), pump2_ref(outflow), inflow to tank1_ref, lateral inflow_ref]
% """

function data = dataLoad(filename)
    addpath("data"); 
    no_flow_data = 1;
    if contains(filename, ".mat")
        dataT = cell2mat(struct2cell(load(append('.\data\',filename))));
    elseif contains(filename, ".csv")
        dataT = readmatrix(append('\data\',filename))';
    end
    
    if no_flow_data
        dataT(7,:) = NaN;
    end
    
    data = dataT;
end
