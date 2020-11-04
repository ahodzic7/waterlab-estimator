%%Selecting pipes
%We want pipes 100m from either end:
end_dist = 100;

% All pipe lengths
pipe_lengths = [ 4.389  
42.247 
77.795 
94.352 
90.901 
173.418
95.477 
97.302 
79.173 
86.342 
87.067 
95.973 
54.369 
14.428 
11.209 
83.112 
79.657 
19.885 
26.780];
pipe_sum = cumsum(pipe_lengths);
%Selecting pipes
pipe1_index = sum(pipe_sum < 100)
pipe2_index = sum(pipe_sum < pipe_sum(end)/3)
pipe3_index = sum(pipe_sum < pipe_sum(end)*2/3)
pipe4_index = sum(pipe_sum < pipe_sum(end) - 100)