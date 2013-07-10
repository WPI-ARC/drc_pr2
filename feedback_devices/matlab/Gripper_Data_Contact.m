close all;
clear;
clc;

format bank;
format compact;

temp = csvread('gripper_data.csv');
data = temp(50:200, :);
figure(1);

B = linspace(1,size(data,1),size(data,1));

figure(1);
subplot(3,1,1);
hold all;

plot(B, data(:,1));
plot(B, data(:,2));
plot(B, data(:,3));
plot(B, data(:,4));
plot(B, data(:,5));
plot(B, data(:,6));
plot(B, data(:,7));
plot(B, data(:,8));
plot(B, data(:,9));
plot(B, data(:,10));
plot(B, data(:,11));
plot(B, data(:,12));
plot(B, data(:,13));
plot(B, data(:,14));
plot(B, data(:,15));

subplot(3,1,2);
hold all

plot(B, data(:,1) - mean(data(50:100, 1)));
plot(B, data(:,2) - mean(data(50:100, 2)));
plot(B, data(:,3)- mean(data(50:100, 3)));
plot(B, data(:,4)- mean(data(50:100, 4)));
plot(B, data(:,5)- mean(data(50:100, 5)));
plot(B, data(:,6)- mean(data(50:100, 6)));
plot(B, data(:,7)- mean(data(50:100, 7)));
plot(B, data(:,8)- mean(data(50:100, 8)));
plot(B, data(:,9)- mean(data(50:100, 9)));
plot(B, data(:,10)- mean(data(50:100, 10)));
plot(B, data(:,11)- mean(data(50:100, 11)));
plot(B, data(:,12)- mean(data(50:100, 12)));
plot(B, data(:,13)- mean(data(50:100, 13)));
plot(B, data(:,14)- mean(data(50:100, 14)));
plot(B, data(:,15) - mean(data(50:100, 15)));

subplot(3,1,3);
hold all

for i = 2:size(data,1),
    data_d(i-1,:) = data(i, :) - data(i-1, :);
end

C = linspace(1,size(data_d,1),size(data_d,1));

plot(C, data_d(:,1));
plot(C, data_d(:,2));
plot(C, data_d(:,3));
plot(C, data_d(:,4));
plot(C, data_d(:,5));
plot(C, data_d(:,6));
plot(C, data_d(:,7));
plot(C, data_d(:,8));
plot(C, data_d(:,9));
plot(C, data_d(:,10));
plot(C, data_d(:,11));
plot(C, data_d(:,12));
plot(C, data_d(:,13));
plot(C, data_d(:,14));
plot(C, data_d(:,15));
