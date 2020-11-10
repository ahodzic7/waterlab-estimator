clear all;
load('Pump_test_3.mat');
data = ans;

Q = data(10,:);         %L/min
Q = (Q*100^3)/60;       %mm^3/s
height = data(8,:);     %mm
tank_area = data(11,1);

dt = 0.5;

height_model = -(1/tank_area)*cumsum(Q)*dt;
figure
plot(height);
hold on;
plot(height_model+height(3));

