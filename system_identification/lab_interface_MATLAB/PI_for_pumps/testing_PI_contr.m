Kp = 7.4304;
Ki = 30.1148;

s = tf('s');

a = tf1*(Ki/s+Kp);
b = 1;
sys = a/(1+a*b);
percent = sys/tf1;
flow = ones(100,1)*2;
flow(1:10,1) = 0;
flow = [flow; ones(100,1)*6; ones(100,1)*3; ones(100,1)*0; ones(100,1)*10];
figure
lsim(sys,flow,1:500);
figure
lsim(percent,flow,1:500);