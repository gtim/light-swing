%% Plot Accelerometer Output
% Plots the accelerometer and gyrometer readings collected by the
% acceleration-logger.ino sketch. The purpose is to decide on a signal
% processing approach. The third plot, acceleration and angular velocity 
% magnitudes, should be most useful for that purpose.

clear
close all

%% Read stored sensor readings
% uncomment relevant filename

csv_filename = 'output-01-soft-swinging.csv';
%csv_filename = 'output-02-pushed-swinging.csv';
%csv_filename = 'output-03-in-lap.csv';

sensordata = csvread(csv_filename);
t    = sensordata(:,1);
acc  = sensordata(:,2:4);
gyro = sensordata(:,5:7);

%% Crop time axis

t_max = 30e3; % 30s
acc  = acc(  t < t_max, : );
gyro = gyro( t < t_max, : );
t    = t(    t < t_max );

%% Plot accelerometer component readings

figure(1);
plot( t*1e-3, acc )
title( [ 'Acceleration components, ' csv_filename ] );
xlabel( 'time / s' );
ylabel( 'acceleration / ms^{-2}' );
legend('x-component', 'y-component', 'z-component');

%% Plot gyrometer component readings

figure(2);
plot( t*1e-3, gyro )
title( [ 'Ang. vel. components, ' csv_filename  ] );
xlabel( 'time / s' );
ylabel( 'angular velocity / rads^{-1}' );
legend('x-component', 'y-component', 'z-component');

%% Plot magnitudes of acceleration and angular velocity

figure(3);
acc_mag  = sqrt( sum(acc.^2, 2) );
gyro_mag = sqrt( sum(gyro.^2,2) );
[h_plotyy,~,~] = plotyy( t*1e-3, acc_mag , t*1e-3, gyro_mag );
xlabel( 'time / s' );
title( ['Acceleration and ang.vel, ' csv_filename ] );
ylabel( h_plotyy(1), 'acceleration / ms^{-2}' );
ylabel( h_plotyy(2), 'angular velocity / rads^{-1}' );
legend( 'Acceleration', 'Angular velocity' );