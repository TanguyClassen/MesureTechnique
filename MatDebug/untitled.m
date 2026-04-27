
% Parameters (edit)
filename = 'exp_20260424_142006_accel.csv';
signalCol = 1;   % column index (after removing first column) that contains the filtered signal
Fs = 1000;       % sampling frequency in Hz

% Read CSV and remove first column
M = readmatrix(filename);      % numeric read; use readtable if mixed types
M(:,1) = [];                   % remove first column

% Optionally save the trimmed CSV
writematrix(M, 'data_trimmed.csv');

% Extract filtered signal (column vector)
x = M(:, signalCol);
N = length(x);

% Compute FFT (n-point; use nextpow2 for zero-padding if desired)
nfft = 2^nextpow2(N);
X = fft(x, nfft);

% Frequency vector (one-sided)
f = Fs*(0:(nfft/2))/nfft;
X_mag = abs(X(1:nfft/2+1))/N;      % normalized magnitude
X_mag(2:end-1) = 2*X_mag(2:end-1); % scale for one-sided spectrum

% Plot magnitude spectrum
figure;
plot(f, X_mag, 'LineWidth', 1.2);
xlabel('Frequency (Hz)');
ylabel('|X(f)|');
title('Single-Sided Amplitude Spectrum');

% (Optional) Save spectrum to CSV
spec = [f(:), X_mag(:)];
writematrix(spec, 'spectrum.csv');