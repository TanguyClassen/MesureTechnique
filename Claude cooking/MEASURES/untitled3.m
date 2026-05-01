% ═══════════════════════════════════════════════════════════════════════════════
% FFT on a user-defined time window
% ═══════════════════════════════════════════════════════════════════════════════

clear; clc; close all;

% ─────────────────────────────────────────────────────────────────────────────
%  ① FILE
% ─────────────────────────────────────────────────────────────────────────────
CSV_FILE = 'M1/M1_1.csv';
SIGNAL   = 'norm_accel';   % weight_kg | angle_deg | angle_deg | accel_x | accel_y | accel_z
SOURCE   = 'accel';       % weight    | video       | video

% ─────────────────────────────────────────────────────────────────────────────
%  ② TIME WINDOW
% ─────────────────────────────────────────────────────────────────────────────
T_MIN = 10;    % [s]  start of analysis window
T_MAX = 40;   % [s]  end   of analysis window

% ─────────────────────────────────────────────────────────────────────────────
%  ③ FFT PARAMETERS
% ─────────────────────────────────────────────────────────────────────────────
WINDOW_TYPE = 'blackman';  % blackman | hann | hamming | rectwin (no window)
ZERO_PAD    = 4;           % zero-padding factor (1 = none, 2 = 2×, 4 = 4×)
                           % higher → smoother/denser spectrum, same resolution
FREQ_MIN    = 0.0;         % [Hz] plot x-axis from
FREQ_MAX    = 5.0;         % [Hz] plot x-axis to
N_PEAKS     = 3;           % how many dominant peaks to label
DETREND_ON  = true;        % remove linear trend before FFT

% ─────────────────────────────────────────────────────────────────────────────
%  ④ OPTIONAL PRE-FILTER  (applied before FFT, set PREFILTER=false to skip)
% ─────────────────────────────────────────────────────────────────────────────
PREFILTER    = false;
FILTER_TYPE  = 'bandpass'; % low | bandpass
FILTER_FC    = [0.3 4.0];  % [Hz]  one value for low-pass, two for band-pass
FILTER_ORDER = 4;

% ═══════════════════════════════════════════════════════════════════════════════
%  LOAD
% ═══════════════════════════════════════════════════════════════════════════════
BASE = fileparts(mfilename('fullpath'));
T    = readtable(fullfile(BASE, CSV_FILE), 'TextType','string');
src  = lower(string(T.source));
D    = T(src == lower(SOURCE), :);

if height(D) == 0
    error('No rows found for source="%s" in %s', SOURCE, CSV_FILE);
end
if ~ismember(SIGNAL, D.Properties.VariableNames)
    error('Column "%s" not found. Available: %s', SIGNAL, strjoin(D.Properties.VariableNames,', '));
end

% Time vector
t = double(D.time_s) - double(D.time_s(1));
y = double(D.(SIGNAL));

% Remove NaN
ok = ~isnan(t) & ~isnan(y);
t  = t(ok); y = y(ok);

% Measured sample rate
dt = median(diff(t));
Fs = 1/dt;
fprintf('File     : %s\n', CSV_FILE);
fprintf('Signal   : %s  [%s]\n', SIGNAL, SOURCE);
fprintf('Full SR  : %.2f Hz   Total duration: %.2f s\n', Fs, t(end));

% ─────────────────────────────────────────────────────────────────────────────
%  CROP TO WINDOW
% ─────────────────────────────────────────────────────────────────────────────
mask = t >= T_MIN & t <= T_MAX;
if sum(mask) < 8
    error('Window [%.1f – %.1f s] contains fewer than 8 samples. Adjust T_MIN/T_MAX.', T_MIN, T_MAX);
end
t = t(mask);
y = y(mask);
fprintf('Window   : %.2f – %.2f s   (%d samples)\n', T_MIN, T_MAX, numel(y));

% ─────────────────────────────────────────────────────────────────────────────
%  DETREND
% ─────────────────────────────────────────────────────────────────────────────
if DETREND_ON
    y = detrend(y, 1);   % remove linear trend (DC + slope)
end

% ─────────────────────────────────────────────────────────────────────────────
%  OPTIONAL PRE-FILTER
% ─────────────────────────────────────────────────────────────────────────────
y_plot = y;   % keep un-filtered copy for time-series plot
if PREFILTER
    switch lower(FILTER_TYPE)
        case 'low'
            [b,a] = butter(FILTER_ORDER, FILTER_FC(1)/(Fs/2), 'low');
        case 'bandpass'
            [b,a] = butter(FILTER_ORDER, FILTER_FC/(Fs/2), 'bandpass');
        otherwise
            error('FILTER_TYPE must be "low" or "bandpass"');
    end
    y = filtfilt(b, a, y);
    fprintf('Pre-filter: %s  fc=%s Hz  order=%d\n', FILTER_TYPE, mat2str(FILTER_FC), FILTER_ORDER);
else
    fprintf('Pre-filter: none\n');
end

% ─────────────────────────────────────────────────────────────────────────────
%  FFT
% ─────────────────────────────────────────────────────────────────────────────
N    = numel(y);
Nfft = N * ZERO_PAD;   % zero-padded length

switch lower(WINDOW_TYPE)
    case 'blackman'; w = blackman(N);
    case 'hann';     w = hann(N);
    case 'hamming';  w = hamming(N);
    case 'rectwin';  w = ones(N,1);
    otherwise;       error('WINDOW_TYPE: blackman | hann | hamming | rectwin');
end

% Normalise window so PSD is amplitude-independent of window choice
w = w / mean(w);

y_win = (y - mean(y)) .* w;
Y     = fft(y_win, Nfft);
P2    = abs(Y/Nfft).^2;
P     = [P2(1); 2*P2(2:floor(Nfft/2)+1)];
f     = Fs*(0:floor(Nfft/2))'/Nfft;

df    = f(2) - f(1);   % frequency resolution
fprintf('Window   : %s   zero-pad ×%d\n', WINDOW_TYPE, ZERO_PAD);
fprintf('Freq res : %.4f Hz\n', df);

% Find top N peaks
mask_f = f >= FREQ_MIN & f <= FREQ_MAX;
f_plot = f(mask_f); P_plot = P(mask_f);

[pks, locs] = findpeaks(P_plot, 'SortStr','descend', 'NPeaks', N_PEAKS, 'MinPeakProminence', max(P_plot)*0.05);
fprintf('\nTop %d peaks in [%.2f – %.2f Hz]:\n', N_PEAKS, FREQ_MIN, FREQ_MAX);
for k = 1:numel(locs)
    fprintf('  #%d  %.4f Hz   PSD = %.4g\n', k, f_plot(locs(k)), pks(k));
end

% ═══════════════════════════════════════════════════════════════════════════════
%  PLOT
% ═══════════════════════════════════════════════════════════════════════════════
figure('Name','FFT Window Analysis','Color',[0.06 0.07 0.09],'Position',[100 80 1200 700]);

sgtitle(sprintf('FFT — %s  |  %s  |  window [%.1f – %.1f s]', ...
        CSV_FILE, SIGNAL, T_MIN, T_MAX), 'Color','w','FontSize',12,'FontWeight','bold');

% ── Top: time series with window highlighted ──────────────────────────────────
ax1 = subplot(2,1,1); dark_ax(ax1); hold on;

% Full signal
plot(t + T_MIN, y_plot + mean(double(D.(SIGNAL)(ok))), ...   % restore mean for context
     'Color',[0.5 0.5 0.6 0.5], 'LineWidth',0.8, 'DisplayName','Signal in window');

% Filtered (if applied)
if PREFILTER
    plot(t + T_MIN, y + mean(double(D.(SIGNAL)(ok))), ...
         'Color',[0.4 0.75 1], 'LineWidth',1.6, 'DisplayName','Pre-filtered');
end

% Window shading
yl = ylim;
fill([T_MIN T_MAX T_MAX T_MIN], [yl(1) yl(1) yl(2) yl(2)], ...
     [0.94 0.65 0.14], 'FaceAlpha',0.08, 'EdgeColor',[0.94 0.65 0.14 0.4], ...
     'LineStyle','--', 'DisplayName','Analysis window');

ylabel(SIGNAL, 'Interpreter','none');
xlabel('Time (s)');
title(sprintf('Time series   SR=%.0f Hz   window=%.1f s   N=%d pts', Fs, T_MAX-T_MIN, N),'Color','w');
legend('TextColor','w','Color',[0.12 0.13 0.2],'Location','best','FontSize',8);

% ── Bottom: FFT ───────────────────────────────────────────────────────────────
ax2 = subplot(2,1,2); dark_ax(ax2); hold on;

% Filled spectrum
area(f_plot, P_plot, 'FaceColor',[0.4 0.6 1], 'FaceAlpha',0.3, ...
     'EdgeColor',[0.4 0.6 1], 'LineWidth',1.2);

% Peak markers
colors_pk = {[1 0.4 0.4],[0.4 1 0.6],[1 0.85 0.3]};
for k = 1:numel(locs)
    col = colors_pk{min(k,3)};
    xline(f_plot(locs(k)), '--', 'Color',[col 0.8], 'LineWidth',1.2);
    text(f_plot(locs(k)), pks(k), sprintf('  #%d: %.4f Hz', k, f_plot(locs(k))), ...
         'Color',col,'FontSize',9,'FontWeight','bold','VerticalAlignment','bottom');
end

% Axis labels
xlabel('Frequency (Hz)');
ylabel('PSD');
title(sprintf('FFT   window=%s   zero-pad×%d   Δf=%.4f Hz', WINDOW_TYPE, ZERO_PAD, df),'Color','w');
xlim([FREQ_MIN FREQ_MAX]);

% Param box
info = sprintf('T: [%.1f – %.1f s]  |  N=%d  |  Nfft=%d  |  Δf=%.4f Hz  |  win=%s  |  pad×%d  |  filter=%s', ...
               T_MIN, T_MAX, N, Nfft, df, WINDOW_TYPE, ZERO_PAD, string(PREFILTER).lower);
annotation('textbox',[0.01 0.01 0.98 0.025],'String',info,'Color',[0.5 0.5 0.5],...
           'FontSize',7,'EdgeColor','none','BackgroundColor','none','Interpreter','none');

% ─────────────────────────────────────────────────────────────────────────────
function dark_ax(ax)
    set(ax,'Color',[0.13 0.15 0.22],'XColor',[0.6 0.62 0.7],...
           'YColor',[0.6 0.62 0.7],'GridColor',[0.25 0.27 0.35],...
           'GridAlpha',0.5,'FontSize',8,'Box','off');
    grid(ax,'on');
end