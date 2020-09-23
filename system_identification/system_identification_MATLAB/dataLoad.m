addpath("data")

%% identification data                                  
load('.\data\g_flow_1');                       
load('.\data\g_level_1');   

load('.\data\g_flow_2');                       
load('.\data\g_level_2'); 

load('.\data\g_flow_3');                       
load('.\data\g_level_3'); 

load('.\data\g_flow_4');                       
load('.\data\g_level_4'); 

load('.\data\g_flow_5');                       
load('.\data\g_level_5'); 

load('.\data\g_flow_6');                       
load('.\data\g_level_6'); 

load('.\data\g_flow_7');                       
load('.\data\g_level_7');

%% datasets

g_flow = [g_flow_1, g_flow_2, g_flow_3, g_flow_4, g_flow_5, g_flow_6, g_flow_7];
g_level = [g_level_1, g_level_2, g_level_3, g_level_4, g_level_5, g_level_6, g_level_7];