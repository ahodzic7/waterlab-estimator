% Loading the data into a .mat file
% 
% The data must be in the following form:
% 
% [...
% 1-Time; ... 
% 2-tank1_depth; ...
% 3-pipe1_height; 4-pipe2_height; 5-pipe3_height; 6-pipe4_depth; ...
% 7-tank2_inflow; 8-tank2_depth; 
% 9-pump1_flow; 10-pump2_flow; ...
% 11-tank2_area ...
% ]

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
