Kp = 0.20276;
Ki = 0.017453;

s = tf('s');

a = tf1*(Ki/s+Kp);
b = 1;
sys = a/(1+a*b);
flow = ones(10,1)*2;
flow(1:10,1) = 0;
flow = [flow; ones(800,1)*12; ];
figure
lsim(sys,flow,1:810);