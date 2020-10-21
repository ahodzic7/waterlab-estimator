Kp = 0.24955;
Ki = 1.4929;

s = tf('s');

a = tf1*(Ki/s+Kp);
b = 1;
sys = a/(1+a*b);
flow = ones(10,1)*2;
flow(1:10,1) = 0;
flow = [flow; ones(10,1)*6; ones(10,1)*3; ones(10,1)*0; ones(10,1)*10];
figure
lsim(sys,flow,1:50);