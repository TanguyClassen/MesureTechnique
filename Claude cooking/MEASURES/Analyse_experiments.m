% ═══════════════════════════════════════════════════════════════════════════════
% ME-301 — Water Jet Pendulum — Multi-Power Experiment Analysis
% ═══════════════════════════════════════════════════════════════════════════════
%
% Actual folder structure:
%   M1/  →  low power
%       M1_1.csv, M1_2.csv …          compiled full exports (from Python app)
%       measure1_1_weight.csv          individual weight stream
%       measure1_1_accel.csv           individual accel stream
%       measure1_1.mov                 video (not read here)
%       …
%   M2/  →  50% power   (measure50_x_weight/accel.csv, M2_x.csv)
%   M3/  →  100% power  (measure100_x_weight/accel.csv, M3_x.csv)
%
% The script works even if some folders have no compiled CSVs yet.
% ═══════════════════════════════════════════════════════════════════════════════

clear; clc; close all;
fprintf('ME-301 Water Jet Analysis — starting\n\n');

% ─────────────────────────────────────────────────────────────────────────────
%  USER PARAMETERS
% ─────────────────────────────────────────────────────────────────────────────
BASE_DIR     = fileparts(mfilename('fullpath'));
EXPERIMENTS  = {'M1','M2','M3'};
POWER_LABELS = {'Min power (1%)','50% power','100% power'};
COLORS       = {[0.31 0.56 0.97], [0.24 0.81 0.56], [0.94 0.65 0.14]};

BUTTER_ORDER  = 4;
BUTTER_FC_W  = 0.5;    % was 5.0  — weight is all sub-0.1 Hz
BUTTER_FC_A  = 15.0;   % was 20.0 — fine as-is
BUTTER_FC_TH = 0.0;    % signal: use band-pass instead (see below)
FFT_MAX_FREQ = 5.0;    % for angle plot: zoom into 0–5 Hz is enough
TRANSIENT_S   = 1.0;    % seconds to discard at start of each run

% ─────────────────────────────────────────────────────────────────────────────
%  LOAD DATA — handle partial data gracefully
% ─────────────────────────────────────────────────────────────────────────────
EXP = struct();

for e = 1:numel(EXPERIMENTS)
    tag   = EXPERIMENTS{e};
    label = POWER_LABELS{e};
    col   = COLORS{e};
    folder= fullfile(BASE_DIR, tag);

    EXP(e).tag   = tag;
    EXP(e).label = label;
    EXP(e).color = col;
    EXP(e).has_data = false;

    % Empty accumulators
    W_all = table(); A_all = table(); V_all = table();
    sync_t_all = [];
    run_W = {}; run_A = {};

    if ~isfolder(folder)
        fprintf('  [%s] folder not found — skipping\n', tag);
        EXP(e).W=[]; EXP(e).A=[]; EXP(e).V=[];
        EXP(e).W_filt=[]; EXP(e).A_filt=[]; EXP(e).V_filt=[];
        EXP(e).sr_w=0; EXP(e).sr_a=0; EXP(e).sr_v=0;
        EXP(e).sync_t=[]; EXP(e).run_W={}; EXP(e).run_A={};
        continue;
    end

    % ── 1. Load compiled CSVs  (M1_1.csv, M1_2.csv …) ───────────────────────
    compiled_files = dir(fullfile(folder, [tag '_*.csv']));
    fprintf('  [%s] %d compiled CSV(s) found\n', tag, numel(compiled_files));

    for k = 1:numel(compiled_files)
        fpath = fullfile(compiled_files(k).folder, compiled_files(k).name);
        try
            T = readtable(fpath, 'TextType','string');
            % Remove rows where source column is missing
            if ~ismember('source', T.Properties.VariableNames)
                fprintf('    WARNING: %s has no "source" column — skipping\n', compiled_files(k).name);
                continue;
            end
            src = lower(string(T.source));
            Wk = T(src == "weight", :);
            Ak = T(src == "accel",  :);
            Vk = T(src == "video",  :);

            % Remove transient
            Wk = skip_transient(Wk, TRANSIENT_S);
            Ak = skip_transient(Ak, TRANSIENT_S);
            Vk = skip_transient(Vk, TRANSIENT_S);

            % Accumulate
            W_all = safe_vcat(W_all, Wk);
            A_all = safe_vcat(A_all, Ak);
            V_all = safe_vcat(V_all, Vk);

            % Sync events from this run
            if ismember('sync', T.Properties.VariableNames)
                sr = T(T.sync == 1, :);
                sync_t_all = [sync_t_all; sr.time_s]; %#ok<AGROW>
            end
            fprintf('    Loaded %s — %dW %dA %dV rows\n', ...
                    compiled_files(k).name, height(Wk), height(Ak), height(Vk));
        catch ME2
            fprintf('    ERROR loading %s: %s\n', compiled_files(k).name, ME2.message);
        end
    end

    % ── 2. Load individual weight + accel CSVs ───────────────────────────────
    w_files = dir(fullfile(folder, '*_weight.csv'));
    fprintf('  [%s] %d individual weight CSV(s)\n', tag, numel(w_files));

    for k = 1:numel(w_files)
        try
            Tw = readtable(fullfile(w_files(k).folder, w_files(k).name));
            % Normalise column names
            Tw = fix_columns(Tw);
            Tw = skip_transient(Tw, TRANSIENT_S);
            if height(Tw) > 0; run_W{end+1} = Tw; end %#ok<AGROW>

            % Corresponding accel file
            a_name = strrep(w_files(k).name, '_weight.csv', '_accel.csv');
            a_path = fullfile(w_files(k).folder, a_name);
            if isfile(a_path)
                Ta = readtable(a_path);
                Ta = fix_columns(Ta);
                Ta = skip_transient(Ta, TRANSIENT_S);
                if height(Ta) > 0; run_A{end+1} = Ta; end %#ok<AGROW>
            end
        catch ME2
            fprintf('    ERROR loading %s: %s\n', w_files(k).name, ME2.message);
        end
    end

    % ── 3. Clean NaN rows ────────────────────────────────────────────────────
    if height(W_all) > 0 && ismember('weight_kg', W_all.Properties.VariableNames)
        W_all = W_all(~isnan(W_all.weight_kg), :);
    end
    if height(A_all) > 0 && ismember('norm_accel', A_all.Properties.VariableNames)
        A_all = A_all(~isnan(A_all.norm_accel), :);
    end
    if height(V_all) > 0 && ismember('angle_deg', V_all.Properties.VariableNames)
        V_all = V_all(~isnan(V_all.angle_deg), :);
    end

    % If no compiled CSV, build W_all / A_all from individual runs
    if height(W_all) == 0 && ~isempty(run_W)
        for k = 1:numel(run_W)
            W_all = safe_vcat(W_all, run_W{k});
        end
    end
    if height(A_all) == 0 && ~isempty(run_A)
        for k = 1:numel(run_A)
            A_all = safe_vcat(A_all, run_A{k});
        end
    end

    % ── 4. Sample rates ───────────────────────────────────────────────────────
    sr_w = 0; sr_a = 0; sr_v = 0;
    if height(W_all) > 1 && ismember('time_ms', W_all.Properties.VariableNames)
        W_all.time_s = W_all.time_ms / 1000.0;
        sr_w = eff_sr(W_all.time_s);
    elseif height(W_all) > 1 && ismember('time_s', W_all.Properties.VariableNames)
        sr_w = eff_sr(W_all.time_s);
    end
    if height(A_all) > 1 && ismember('time_ms', A_all.Properties.VariableNames)
        A_all.time_s = A_all.time_ms / 1000.0;
        sr_a = eff_sr(A_all.time_s);
    elseif height(A_all) > 1 && ismember('time_s', A_all.Properties.VariableNames)
        sr_a = eff_sr(A_all.time_s);
    end
    if height(V_all) > 1 && ismember('time_s', V_all.Properties.VariableNames)
        sr_v = eff_sr(V_all.time_s);
    end

    fprintf('    SR — weight: %.1f Hz | accel: %.1f Hz | video: %.1f Hz\n', sr_w, sr_a, sr_v);

    % ── 5. Butterworth filters ────────────────────────────────────────────────
    [W_f, A_f, V_f] = apply_filters(W_all, A_all, V_all, sr_w, sr_a, sr_v, ...
                                     BUTTER_ORDER, BUTTER_FC_W, BUTTER_FC_A, BUTTER_FC_TH);

    EXP(e).W      = W_all; EXP(e).A      = A_all; EXP(e).V      = V_all;
    EXP(e).W_filt = W_f;   EXP(e).A_filt = A_f;   EXP(e).V_filt = V_f;
    EXP(e).sr_w   = sr_w;  EXP(e).sr_a   = sr_a;  EXP(e).sr_v   = sr_v;
    EXP(e).sync_t = sync_t_all;
    EXP(e).run_W  = run_W; EXP(e).run_A  = run_A;
    EXP(e).has_data = (height(W_all) > 0 || height(A_all) > 0);
end

% Which experiments actually have data?
has = arrayfun(@(e) e.has_data, EXP);
active = find(has);
fprintf('\n%d / %d experiments have data: %s\n\n', ...
        sum(has), numel(EXPERIMENTS), strjoin(EXPERIMENTS(active),', '));

if isempty(active)
    error('No data found in any experiment folder. Check BASE_DIR and folder names.');
end

% ─────────────────────────────────────────────────────────────────────────────
%  FIGURE 1 — Raw vs filtered overview
% ─────────────────────────────────────────────────────────────────────────────
fprintf('── Figure 1: Raw data overview ──\n');
fig1 = sfig('Raw Sensor Data', [100 50 1400 820]);

row_sigs = {'weight_kg','norm_accel','angle_deg'};
row_srcs = {'W','A','V'};
row_filt = {'W_filt','A_filt','V_filt'};
row_labs = {'Weight (kg)','|a| (m/s²)','Angle θ (°)'};
row_srs  = {'sr_w','sr_a','sr_v'};

n = numel(active);
for ri = 1:3
    for ci = 1:n
        e  = active(ci);
        ex = EXP(e);
        ax = subplot(3, n, (ri-1)*n + ci);
        sax(ax);
        dat  = ex.(row_srcs{ri});
        datf = ex.(row_filt{ri});
        sig  = row_sigs{ri};
        if istable(dat) && height(dat)>0 && ismember(sig,dat.Properties.VariableNames) && ismember('time_s',dat.Properties.VariableNames)
            plot(dat.time_s,  dat.(sig),  'Color',[ex.color 0.4], 'LineWidth',0.7); hold on;
            plot(datf.time_s, datf.(sig), 'Color', ex.color,      'LineWidth',1.5);
            draw_sync(ax, ex.sync_t);
        else
            text(0.5,0.5,'No data','Units','normalized','HorizontalAlignment','center','Color',[0.5 0.5 0.5]);
        end
        if ri==1; title(sprintf('%s — %s', ex.tag, ex.label),'Color','w','FontSize',9); end
        if ci==1; ylabel(row_labs{ri}); end
        xlabel('Time (s)');
        srl(ax, ex.(row_srs{ri}));
    end
end
sgtitle('Raw (faded) vs Filtered (solid) — All Experiments','Color','w','FontSize',13,'FontWeight','bold');

% ─────────────────────────────────────────────────────────────────────────────
%  FIGURE 2 — Filtered signals overlaid
% ─────────────────────────────────────────────────────────────────────────────
fprintf('── Figure 2: Filtered comparison ──\n');
fig2 = sfig('Filtered Comparison', [120 70 1200 780]);

fc_cutoffs = [BUTTER_FC_W, BUTTER_FC_A, BUTTER_FC_TH];
for ri = 1:3
    ax = subplot(3,1,ri); sax(ax); hold on;
    any_plotted = false;
    for ci = 1:n
        e  = active(ci); ex = EXP(e);
        datf = ex.(row_filt{ri}); sig = row_sigs{ri};
        if istable(datf) && height(datf)>0 && ismember(sig,datf.Properties.VariableNames)
            plot(datf.time_s, datf.(sig), 'Color',ex.color,'LineWidth',1.6,'DisplayName',ex.label);
            any_plotted = true;
        end
    end
    ylabel(row_labs{ri}); xlabel('Time (s)');
    if any_plotted
        legend('show','Location','best','TextColor','w','Color',[0.1 0.1 0.15],'FontSize',8);
    end
    title(sprintf('Filtered — %s   (Butter %d-order, fc=%.0fHz)', ...
          row_labs{ri}, BUTTER_ORDER, fc_cutoffs(ri)),'Color','w');
end
sgtitle('Filtered Signal Overlay — Power Level Comparison','Color','w','FontSize',13,'FontWeight','bold');

% ─────────────────────────────────────────────────────────────────────────────
%  FIGURE 3 — FFT overlay (one subplot per signal)
% ─────────────────────────────────────────────────────────────────────────────
fprintf('── Figure 3: FFT overlay ──\n');
fig3 = sfig('FFT Overlay', [140 90 1200 750]);

fft_ylabs = {'Weight PSD  (kg²/Hz)','|a| PSD  (m²s⁻⁴/Hz)','Angle PSD  (°²/Hz)'};

for ri = 1:3
    ax = subplot(3,1,ri); sax(ax); hold on;
    for ci = 1:n
        e  = active(ci); ex = EXP(e);
        datf = ex.(row_filt{ri}); sig = row_sigs{ri};
        if istable(datf) && height(datf)>2 && ismember(sig,datf.Properties.VariableNames)
            [fv,P] = cfft(datf.time_s, datf.(sig));
            if isempty(fv); continue; end
            mask = fv>0 & fv<=FFT_MAX_FREQ;
            plot(fv(mask), P(mask), 'Color',ex.color,'LineWidth',1.4,'DisplayName',ex.label);
            % Mark dominant
            [Pm,ix] = max(P(mask)); fm = fv(mask);
            xline(fm(ix),'--','Color',[ex.color 0.6],'LineWidth',0.9,'HandleVisibility','off');
            text(fm(ix), Pm, sprintf(' %.2fHz',fm(ix)),'Color',ex.color,'FontSize',7,'VerticalAlignment','bottom');
        end
    end
    ylabel(fft_ylabs{ri}); xlabel('Frequency (Hz)');
    legend('show','Location','northeast','TextColor','w','Color',[0.1 0.1 0.15],'FontSize',8);
    title(sprintf('FFT — %s', fft_ylabs{ri}),'Color','w');
end
sgtitle('Spectral Analysis — Power Level Comparison','Color','w','FontSize',13,'FontWeight','bold');

% ─────────────────────────────────────────────────────────────────────────────
%  FIGURE 4 — FFT grid  (signal × experiment)
% ─────────────────────────────────────────────────────────────────────────────
fprintf('── Figure 4: FFT grid per experiment ──\n');
fig4 = sfig('FFT per Experiment', [160 110 1400 800]);

for ri = 1:3
    for ci = 1:n
        e  = active(ci); ex = EXP(e);
        ax = subplot(3, n, (ri-1)*n + ci); sax(ax);
        datf = ex.(row_filt{ri}); sig = row_sigs{ri};
        if istable(datf) && height(datf)>2 && ismember(sig,datf.Properties.VariableNames)
            [fv,P] = cfft(datf.time_s, datf.(sig));
            if ~isempty(fv)
                mask = fv>0 & fv<=FFT_MAX_FREQ;
                area(fv(mask), P(mask),'FaceColor',ex.color,'FaceAlpha',0.35,'EdgeColor',ex.color,'LineWidth',1.1);
                [Pm,ix]=max(P(mask)); fm=fv(mask);
                hold on; xline(fm(ix),'w--','LineWidth',0.8);
                text(fm(ix),Pm*0.92,sprintf(' %.2fHz',fm(ix)),'Color','w','FontSize',7,'VerticalAlignment','top');
            end
        else
            text(0.5,0.5,'No data','Units','normalized','HorizontalAlignment','center','Color',[0.5 0.5 0.5]);
        end
        if ri==1; title(sprintf('%s — %s',ex.tag,ex.label),'Color','w','FontSize',9); end
        if ci==1; ylabel(fft_ylabs{ri}); end
        xlabel('Frequency (Hz)');
        srl(ax, ex.(row_srs{ri}));
    end
end
sgtitle('FFT per Experiment and Signal','Color','w','FontSize',13,'FontWeight','bold');

% ─────────────────────────────────────────────────────────────────────────────
%  FIGURE 5 — Statistics bar charts
% ─────────────────────────────────────────────────────────────────────────────
fprintf('── Figure 5: Statistics ──\n');
fig5 = sfig('Statistics', [180 130 1100 700]);

stat_ops  = {'Mean','Std dev','RMS','Peak |val|'};
S = nan(3, n, 4);   % [signal, experiment, stat]
for ri = 1:3
    for ci = 1:n
        e  = active(ci); ex = EXP(e);
        datf = ex.(row_filt{ri}); sig = row_sigs{ri};
        if istable(datf) && height(datf)>0 && ismember(sig,datf.Properties.VariableNames)
            y = double(datf.(sig)); y = y(~isnan(y));
            if ~isempty(y)
                S(ri,ci,1) = mean(y);
                S(ri,ci,2) = std(y);
                S(ri,ci,3) = rms(y);
                S(ri,ci,4) = max(abs(y));
            end
        end
    end
end

for ri = 1:3
    for op = 1:4
        ax = subplot(3,4,(ri-1)*4+op); sax(ax);
        vals = squeeze(S(ri,:,op));
        b = bar(vals,'FaceColor','flat');
        for ci = 1:n
            b.CData(ci,:) = COLORS{active(ci)};
        end
        set(ax,'XTickLabel',EXPERIMENTS(active),'XTick',1:n);
        if op==1; ylabel(row_labs{ri}); end
        title(stat_ops{op},'Color','w','FontSize',8);
        for ci = 1:n
            if ~isnan(vals(ci))
                text(ci, vals(ci)*1.04, sprintf('%.3g',vals(ci)),...
                     'HorizontalAlignment','center','Color','w','FontSize',7);
            end
        end
        ylim([0, max(vals(~isnan(vals)))*1.2 + eps]);
    end
end
sgtitle('Statistical Summary — Filtered Signals','Color','w','FontSize',13,'FontWeight','bold');

% ─────────────────────────────────────────────────────────────────────────────
%  FIGURE 6 — Individual runs (weight only, since accel CSVs are separate)
% ─────────────────────────────────────────────────────────────────────────────
fprintf('── Figure 6: Individual runs ──\n');
fig6 = sfig('Individual Runs', [200 150 1300 700]);

for ci = 1:n
    e  = active(ci); ex = EXP(e);
    ax = subplot(1,n,ci); sax(ax); hold on;
    cmap = cool(max(numel(ex.run_W),1));
    for k = 1:numel(ex.run_W)
        Tk = ex.run_W{k};
        if ~ismember('weight_kg',Tk.Properties.VariableNames); continue; end
        t_col = 'time_s';
        if ~ismember(t_col,Tk.Properties.VariableNames) && ismember('time_ms',Tk.Properties.VariableNames)
            Tk.time_s = Tk.time_ms/1000; t_col='time_s';
        end
        if ~ismember(t_col,Tk.Properties.VariableNames); continue; end
        t0 = Tk.(t_col)(1);
        plot(Tk.(t_col)-t0, Tk.weight_kg, 'Color',[cmap(k,:) 0.7], ...
             'LineWidth',0.9,'DisplayName',sprintf('Run %d',k));
    end
    % Overlay compiled filtered
    if istable(ex.W_filt) && height(ex.W_filt)>0 && ismember('weight_kg',ex.W_filt.Properties.VariableNames)
        t0 = ex.W_filt.time_s(1);
        plot(ex.W_filt.time_s-t0, ex.W_filt.weight_kg,'w--','LineWidth',1.6,'DisplayName','All runs filtered');
    end
    title(sprintf('%s — %s',ex.tag,ex.label),'Color','w','FontSize',9);
    xlabel('Time (s)'); if ci==1; ylabel('Weight (kg)'); end
    legend('show','Location','best','TextColor','w','Color',[0.1 0.1 0.15],'FontSize',7);
end
sgtitle('Individual Runs — Weight  (filtered overlay in white dashed)','Color','w','FontSize',12,'FontWeight','bold');

% ─────────────────────────────────────────────────────────────────────────────
%  FIGURE 7 — Angle vs Weight scatter (only if video data exists)
% ─────────────────────────────────────────────────────────────────────────────
any_video = any(arrayfun(@(i) istable(EXP(i).V_filt) && height(EXP(i).V_filt)>0, active));
if any_video
    fprintf('── Figure 7: Angle vs Load ──\n');
    fig7 = sfig('Angle vs Load', [220 170 900 600]);
    ax7  = axes('Parent',fig7); sax(ax7); hold on;
    for ci = 1:n
        e=active(ci); ex=EXP(e);
        if ~(istable(ex.V_filt)&&height(ex.V_filt)>0); continue; end
        if ~(istable(ex.W_filt)&&height(ex.W_filt)>0); continue; end
        t_v = ex.V_filt.time_s; t_w = ex.W_filt.time_s;
        th  = double(ex.V_filt.angle_deg);
        w_i = interp1(t_w, double(ex.W_filt.weight_kg), t_v,'linear','extrap');
        ok  = ~isnan(th) & ~isnan(w_i);
        scatter(w_i(ok),th(ok),10,ex.color,'filled','MarkerFaceAlpha',0.5,'DisplayName',ex.label);
        if sum(ok)>2
            c=polyfit(w_i(ok),th(ok),1);
            xf=linspace(min(w_i(ok)),max(w_i(ok)),100);
            plot(xf,polyval(c,xf),'--','Color',ex.color,'LineWidth',1.2,'HandleVisibility','off');
        end
    end
    xlabel('Weight (kg)'); ylabel('Angle θ (°)');
    legend('show','Location','best','TextColor','w','Color',[0.1 0.1 0.15]);
    title('Pendulum angle vs Load — all power levels (dashed = linear fit)','Color','w');
end

% ─────────────────────────────────────────────────────────────────────────────
%  CONSOLE SUMMARY TABLE
% ─────────────────────────────────────────────────────────────────────────────
fprintf('\n══════════════════════════════════════════════════════════════════\n');
fprintf('  SUMMARY TABLE   (filtered signals)\n');
fprintf('══════════════════════════════════════════════════════════════════\n');
fprintf('%-6s %-14s  %-12s %-12s %-12s %-12s\n', ...
        'Exp','Power','Mean W(kg)','RMS |a|','Mean θ(°)','N runs');
fprintf('%s\n', repmat('-',1,72));
for ci = 1:n
    e=active(ci); ex=EXP(e);
    mw=nan; ma=nan; mt=nan;
    if istable(ex.W_filt)&&height(ex.W_filt)>0&&ismember('weight_kg',ex.W_filt.Properties.VariableNames)
        mw=mean(double(ex.W_filt.weight_kg),'omitnan');
    end
    if istable(ex.A_filt)&&height(ex.A_filt)>0&&ismember('norm_accel',ex.A_filt.Properties.VariableNames)
        ma=rms(double(ex.A_filt.norm_accel(~isnan(double(ex.A_filt.norm_accel)))));
    end
    if istable(ex.V_filt)&&height(ex.V_filt)>0&&ismember('angle_deg',ex.V_filt.Properties.VariableNames)
        mt=mean(double(ex.V_filt.angle_deg),'omitnan');
    end
    fprintf('%-6s %-14s  %-12.3f %-12.3f %-12.3f %-12d\n', ...
            ex.tag, ex.label, mw, ma, mt, numel(ex.run_W));
end
fprintf('══════════════════════════════════════════════════════════════════\n');
fprintf('\nAll figures generated successfully.\n');

% ═══════════════════════════════════════════════════════════════════════════════
%  LOCAL FUNCTIONS
% ═══════════════════════════════════════════════════════════════════════════════

function T = skip_transient(T, dt)
    if ~istable(T) || height(T)==0; return; end
    t_col = '';
    if ismember('time_s',  T.Properties.VariableNames); t_col='time_s'; end
    if ismember('time_ms', T.Properties.VariableNames)
        T.time_s = T.time_ms/1000; t_col='time_s';
    end
    if isempty(t_col); return; end
    t0 = T.(t_col)(1);
    T  = T(T.(t_col)-t0 >= dt, :);
end

function T = fix_columns(T)
    % Rename 'time_ms' → keep, add time_s; ensure numeric
    if ismember('time_ms',T.Properties.VariableNames) && ~ismember('time_s',T.Properties.VariableNames)
        T.time_s = T.time_ms / 1000.0;
    end
    % If accel CSV has no norm_accel, compute it
    if ~ismember('norm_accel',T.Properties.VariableNames) && ...
       ismember('accel_x',T.Properties.VariableNames)
        ax=double(T.accel_x); ay=double(T.accel_y); az=double(T.accel_z);
        T.norm_accel = sqrt(ax.^2+ay.^2+az.^2);
    end
end

function C = safe_vcat(A, B)
    if ~istable(A)||height(A)==0; C=B; return; end
    if ~istable(B)||height(B)==0; C=A; return; end
    % Keep only common columns
    cols = intersect(A.Properties.VariableNames, B.Properties.VariableNames);
    C = [A(:,cols); B(:,cols)];
end

function sr = eff_sr(t)
    t=double(t); if numel(t)<2; sr=0; return; end
    dt=diff(t); dt=dt(dt>0);
    if isempty(dt); sr=0; else; sr=1/median(dt); end
end

function [W_f,A_f,V_f] = apply_filters(W,A,V,sr_w,sr_a,sr_v,ord,fcw,fca,fcth)
    W_f=W; A_f=A; V_f=V;
    if istable(W)&&height(W)>ord*6&&sr_w>2*fcw&&ismember('weight_kg',W.Properties.VariableNames)
        [b,a]=butter(ord,fcw/(sr_w/2),'low');
        W_f.weight_kg=filtfilt(b,a,double(W.weight_kg));
    end
    if istable(A)&&height(A)>ord*6&&sr_a>2*fca&&ismember('norm_accel',A.Properties.VariableNames)
        [b,a]=butter(ord,fca/(sr_a/2),'low');
        A_f.norm_accel=filtfilt(b,a,double(A.norm_accel));
        for col={'accel_x','accel_y','accel_z'}
            if ismember(col{1},A.Properties.VariableNames)
                A_f.(col{1})=filtfilt(b,a,double(A.(col{1})));
            end
        end
    end
    if istable(V) && height(V)>ord*6 && sr_v>8 && ismember('angle_deg',V.Properties.VariableNames)
    [b,a] = butter(ord, [0.3 4.0]/(sr_v/2), 'bandpass');
    V_f.angle_deg = filtfilt(b, a, double(V.angle_deg));
    end
end

function [f,P] = cfft(t,y)
% Hann window — for weight and accel
    y=double(y); t=double(t); ok=~isnan(y); y=y(ok); t=t(ok);
    N=numel(y); if N<8; f=[]; P=[]; return; end
    dt=median(diff(t)); Fs=1/dt;
    w = hann(N);
    Y = fft((y - mean(y)) .* w);
    P2=abs(Y/N).^2;
    P=[P2(1); 2*P2(2:floor(N/2)+1)];
    f=Fs*(0:floor(N/2))'/N;
end

function [f,P] = cfft_blackman(t,y)
% Blackman window — for angle (narrow-band pendulum peak)
    y=double(y); t=double(t); ok=~isnan(y); y=y(ok); t=t(ok);
    N=numel(y); if N<8; f=[]; P=[]; return; end
    dt=median(diff(t)); Fs=1/dt;
    w = blackman(N);
    Y = fft((y - mean(y)) .* w);
    P2=abs(Y/N).^2;
    P=[P2(1); 2*P2(2:floor(N/2)+1)];
    f=Fs*(0:floor(N/2))'/N;
end

function draw_sync(ax,sync_t)
    if isempty(sync_t); return; end
    hold(ax,'on');
    for i=1:numel(sync_t)
        xline(ax,sync_t(i),'--','Color',[0.95 0.65 0.14 0.65],'LineWidth',0.9);
    end
end

function srl(ax,sr)
    text(ax,0.99,0.97,sprintf('SR:%.0fHz',sr),'Units','normalized',...
         'HorizontalAlignment','right','VerticalAlignment','top',...
         'Color',[0.55 0.55 0.55],'FontSize',7);
end

function fig=sfig(name,pos)
    fig=figure('Name',name,'NumberTitle','off','Color',[0.06 0.07 0.09],'Position',pos);
end

function sax(ax)
    set(ax,'Color',[0.13 0.15 0.22],'XColor',[0.55 0.57 0.65],...
           'YColor',[0.55 0.57 0.65],'GridColor',[0.25 0.27 0.35],...
           'GridAlpha',0.5,'FontSize',8,'Box','off');
    grid(ax,'on');
end