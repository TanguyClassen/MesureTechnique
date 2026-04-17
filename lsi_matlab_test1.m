clear;
clc;
close all;

port = "/dev/cu.usbmodem1101";      % A adapter
baudrate = 9600;

s = serialport(port, baudrate);
flush(s);

N = 200;

t = zeros(N,1);
ax = zeros(N,1);
ay = zeros(N,1);
az = zeros(N,1);
amag = zeros(N,1);

pause(2);
flush(s);

k = 1;
while k <= N
    line = strtrim(readline(s));

    if line == ""
        continue;
    end

    if contains(line, "MMA8451 found!") || contains(line, "time_ms") || contains(line, "Could not find")
        continue;
    end

    values = split(line, ",");

    if numel(values) == 5
        t(k)    = str2double(values(1)) / 1000;
        ax(k)   = str2double(values(2));
        ay(k)   = str2double(values(3));
        az(k)   = str2double(values(4));
        amag(k) = str2double(values(5));
        k = k + 1;
    end
end

figure;
plot(t, ax, 'LineWidth', 1.2); hold on;
plot(t, ay, 'LineWidth', 1.2);
plot(t, az, 'LineWidth', 1.2);
plot(t, amag, 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
legend('ax','ay','az','|a|');
title('MMA8451 measurements');

clear s;