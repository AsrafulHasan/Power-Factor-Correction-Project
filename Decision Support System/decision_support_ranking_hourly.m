%% decision_support_ranking_hourly.m  (v2 — hour-averaged losses)
%
%  PURPOSE: Rank capacitor-design scenarios by REAL multi-hour
%  performance. Extends the v1 hourly-compliance ranking by also
%  averaging LOSSES across the same 4 representative hours, instead of
%  using peak-load losses alone. Peak-load-only losses understated C3's
%  real advantage: C3 uses far less capacitance most hours (its
%  continuous controller only injects what's needed), which only shows
%  up when losses are evaluated across hours, not at a single point.
%
%  C3 IS NOT A FIXED NAMEPLATE SCENARIO. Its actual injected Mvar is
%  found HOUR BY HOUR by the same continuous-control search PowerWorld
%  performs: find the Mvar (0 to Nominal) that drives its regulated bus
%  (692) to the target voltage (1.00 pu), saturating at Nominal if
%  unreachable. find_continuous_q below reproduces that search using
%  the fast, topology-built-once solver pair (bfs_topology_build.m +
%  bfs_solve_v2droop.m) instead of the slower file-reading solver, since
%  this now runs the search once per hour per scenario.
%
%  DATA SOURCES:
%    hourly_results.csv      -- real PF/voltage compliance per hour
%    capacitor_scenarios.csv -- C1/C2 fixed nameplate placements
%    fault_data.csv          -- peak-load fault-current results
%    load_data_hour{H}.csv   -- per-bus load at each test hour
%
%  VERIFY BEFORE TRUSTING: hourly_results.csv reflects real PowerWorld
%  results confirmed this session -- cross-check against your own notes
%  before using this for your final report.

clear; clc;

%% ============================= CONFIG =============================
hourlyCSV     = 'hourly_results.csv';
scenarioCSV   = 'capacitor_scenarios.csv';
faultCSV      = 'fault_data.csv';
branchCSV     = 'branch_data.csv';
testHours     = [1, 9, 12, 18];
loadCSVprefix = 'load_data_hour';
slackBus      = 650;
baseMVA       = 5.0;
PF_target     = 0.95;
Vband         = [0.95, 1.05];

C3_bus        = 692;
C3_target     = 1.00;
C3_nominalMax = 1.3442;

costPerKvar_low  = 8;
costPerKvar_high = 15;

% C3 additionally needs automatic voltage-based switching hardware (a
% capacitor-bank controller with voltage sensing and automatic control
% logic) -- C1/C2 are plain fixed banks and need none of this. Sourced
% from an industry capacitor-controller pricing survey (Accio.com
% market data, 2025): feature-rich controllers with voltage-based
% automatic control fall in a "Premium Tier" of $1,600-$7,000/unit.
% Using the midpoint as an illustrative estimate, same honesty standard
% as the $8-15/kVAr capacitor-cost range above -- this is NOT a formal
% vendor quote for this specific project, and should be replaced with a
% real quote before being treated as final.
C3_controllerCostUSD = 4300;

weights = struct('complianceRate', 0.35, 'avgPFmargin', 0.15, ...
                  'losses', 0.20, 'faultMargin', 0.15, 'cost', 0.15);

%% ======================= INPUT VALIDATION ==========================
for f = {hourlyCSV, scenarioCSV, faultCSV, branchCSV}
    if ~isfile(f{1})
        error('decision_support_ranking_hourly:fileNotFound', 'Required file not found: %s', f{1});
    end
end
hourlyLoadCSVs = containers.Map('KeyType','double','ValueType','any');
for h = testHours
    p = sprintf('%s%d.csv', loadCSVprefix, h);
    if ~isfile(p)
        error('decision_support_ranking_hourly:fileNotFound', 'Missing hourly load file: %s', p);
    end
    hourlyLoadCSVs(h) = p;
end

wsum = weights.complianceRate + weights.avgPFmargin + weights.losses + weights.faultMargin + weights.cost;
if abs(wsum - 1.0) > 1e-9
    error('decision_support_ranking_hourly:badWeights', 'Weights must sum to 1.0 (currently %.6f).', wsum);
end

hourlyTbl = readtable(hourlyCSV);
scenarioNames = unique(hourlyTbl.Scenario, 'stable');
scenarioTbl = readtable(scenarioCSV);

%% =================== BUILD TOPOLOGY ONCE PER HOUR ===================
fprintf('Building topology for each test hour once (fast solver setup)...\n');
topoByHour = containers.Map('KeyType','double','ValueType','any');
for h = testHours
    topoByHour(h) = bfs_topology_build(branchCSV, hourlyLoadCSVs(h), slackBus, baseMVA);
end
fprintf('Done.\n\n');

%% ================== SECTION 1: HOURLY COMPLIANCE ===================
fprintf('=== Section 1: real multi-hour compliance ===\n\n');
complianceRate = zeros(length(scenarioNames),1);
avgPFmargin = zeros(length(scenarioNames),1);

for i = 1:length(scenarioNames)
    name = scenarioNames{i};
    rows = hourlyTbl(strcmp(hourlyTbl.Scenario, name), :);
    nHours = height(rows);
    nPass = 0; pfMarginsPassing = [];
    fprintf('%s:\n', name);
    for r = 1:nHours
        pf = rows.PF(r); minV = rows.MinV_pu(r); maxV = rows.MaxV_pu(r); h = rows.Hour(r);
        passPF = pf >= PF_target;
        passV  = (minV >= Vband(1)) && (maxV <= Vband(2));
        pass = passPF && passV;
        if pass, nPass = nPass + 1; pfMarginsPassing(end+1) = pf - PF_target; end %#ok<AGROW>
        fprintf('  Hour %2d: PF=%.4f  minV=%.4f  maxV=%.4f  -> %s\n', h, pf, minV, maxV, tern(pass,'PASS','FAIL'));
        if ~passPF
            fprintf('      -> SUGGESTED ACTION: Resize Capacitor, or Split Across Buses (PF below target)\n');
        end
        if ~passV
            if minV < Vband(1)
                fprintf('      -> SUGGESTED ACTION: Relocate Capacitor, or Resize Capacitor (undervoltage)\n');
            end
            if maxV > Vband(2)
                fprintf('      -> SUGGESTED ACTION: Reduce Capacitor Size, or Add Switching (overvoltage)\n');
            end
        end
    end
    complianceRate(i) = nPass / nHours;
    if ~isempty(pfMarginsPassing), avgPFmargin(i) = mean(pfMarginsPassing);
    else, avgPFmargin(i) = min(rows.PF) - PF_target; end
    fprintf('  -> %d/%d hours pass (%.0f%%)\n\n', nPass, nHours, 100*complianceRate(i));
end

%% ================ SECTION 2: HOUR-AVERAGED LOSSES ===================
fprintf('=== Section 2: hour-averaged losses (real per-hour solves) ===\n\n');
avgLossMW = zeros(length(scenarioNames),1);

for i = 1:length(scenarioNames)
    name = scenarioNames{i};
    hourLosses = zeros(length(testHours),1);
    for k = 1:length(testHours)
        h = testHours(k);
        topo = topoByHour(h);
        Pload_sum = sum(topo.Pload) * baseMVA;

        if strcmp(name, 'C3')
            q = find_continuous_q(topo, C3_bus, C3_target, C3_nominalMax);
            capMap = containers.Map('KeyType','double','ValueType','double');
            capMap(C3_bus) = q;
        else
            rows = scenarioTbl(strcmp(scenarioTbl.Name, name), :);
            if isempty(rows)
                error('decision_support_ranking_hourly:unknownScenario', ...
                    'Scenario "%s" has no rows in %s and is not "C3".', name, scenarioCSV);
            end
            capMap = containers.Map('KeyType','double','ValueType','double');
            for r = 1:height(rows), capMap(rows.Bus(r)) = rows.Mvar(r); end
        end

        [~, Psrc, ~, ~] = bfs_solve_v2droop(topo, capMap);
        hourLosses(k) = Psrc - Pload_sum;
    end
    avgLossMW(i) = mean(hourLosses);
    fprintf('%-10s : hourly losses = [%s] MW  ->  avg = %.4f MW\n', name, ...
        strjoin(arrayfun(@(x) sprintf('%.4f',x), hourLosses, 'UniformOutput', false), ', '), avgLossMW(i));
end

%% ================= SECTION 3: PEAK-LOAD FAULT + COST ================
fprintf('\n=== Section 3: fault margin (peak-load, load-insensitive) and cost ===\n\n');
faultTbl = readtable(faultCSV);
faultMarginVals = zeros(length(scenarioNames),1);
nameplateMvar = zeros(length(scenarioNames),1);

for i = 1:length(scenarioNames)
    name = scenarioNames{i};
    scenFault = faultTbl(strcmp(faultTbl.ScenarioName, name), :);
    if isempty(scenFault)
        error('decision_support_ranking_hourly:missingFaultData', 'No fault data for scenario "%s".', name);
    end
    margins = (scenFault.BreakerRatingkA - scenFault.FaultCurrentkA) ./ scenFault.BreakerRatingkA;
    faultMarginVals(i) = min(margins);
    if faultMarginVals(i) < 0
        [worstMargin, worstIdx] = min(margins);
        fprintf('  %-10s : FAULT-DUTY VIOLATION at "%s" (margin %.1f%%)\n', name, scenFault.BusClass{worstIdx}, 100*worstMargin);
        fprintf('      -> SUGGESTED ACTION: Suggest Updated CB (next standard IEC size up from current rating)\n');
    end

    if strcmp(name, 'C3')
        nameplateMvar(i) = C3_nominalMax;  % installed bank size, not hour-varying usage
    else
        rows = scenarioTbl(strcmp(scenarioTbl.Name, name), :);
        nameplateMvar(i) = sum(rows.Mvar);
    end
end
costUSD = nameplateMvar * 1000 * (costPerKvar_low + costPerKvar_high)/2;
c3idx = find(strcmp(scenarioNames, 'C3'));
if ~isempty(c3idx)
    costUSD(c3idx) = costUSD(c3idx) + C3_controllerCostUSD;
    fprintf('C3 cost includes bank ($%.0f) + automatic controller ($%.0f) = $%.0f total\n', ...
        costUSD(c3idx) - C3_controllerCostUSD, C3_controllerCostUSD, costUSD(c3idx));
end

%% ===================== SECTION 4: FINAL RANKING =====================
fprintf('=== Section 4: final ranking ===\n');
fprintf('(weights: compliance=%.2f, PFmargin=%.2f, losses=%.2f, fault=%.2f, cost=%.2f)\n\n', ...
    weights.complianceRate, weights.avgPFmargin, weights.losses, weights.faultMargin, weights.cost);

lossReduction = max(avgLossMW) - avgLossMW;
normComp  = norm01(complianceRate);
normPF    = norm01(avgPFmargin);
normLoss  = norm01(lossReduction);
normFault = norm01(faultMarginVals);
normCost  = norm01(-costUSD);

scores = weights.complianceRate*normComp + weights.avgPFmargin*normPF + ...
         weights.losses*normLoss + weights.faultMargin*normFault + weights.cost*normCost;

for i = 1:length(scenarioNames)
    fprintf('%-10s : compliance=%.0f%%  avgPFmargin=%+.4f  avgLoss=%.4f MW  faultMargin=%.1f%%  cost=$%.0f  -> SCORE=%.4f\n', ...
        scenarioNames{i}, 100*complianceRate(i), avgPFmargin(i), avgLossMW(i), 100*faultMarginVals(i), costUSD(i), scores(i));
end

[bestScore, bestIdx] = max(scores);
fprintf('\n>> Highest-ranked scenario (overall, weighted): %s (score = %.4f)\n', scenarioNames{bestIdx}, bestScore);
fprintf('   This is advisory only -- the engineer makes the final design call.\n');

fprintf('\n=== Per-factor winners (5 factors considered) ===\n');
[~, wComp]  = max(complianceRate);  fprintf('  1. Compliance rate  : %s wins (%.0f%%)\n', scenarioNames{wComp}, 100*complianceRate(wComp));
[~, wPF]    = max(avgPFmargin);     fprintf('  2. Avg PF margin    : %s wins (+%.4f)\n', scenarioNames{wPF}, avgPFmargin(wPF));
[~, wLoss]  = min(avgLossMW);       fprintf('  3. Avg losses       : %s wins (%.4f MW, lowest)\n', scenarioNames{wLoss}, avgLossMW(wLoss));
[~, wFault] = max(faultMarginVals); fprintf('  4. Fault margin     : %s wins (%.1f%%)\n', scenarioNames{wFault}, 100*faultMarginVals(wFault));
[~, wCost]  = min(costUSD);         fprintf('  5. Cost             : %s wins ($%.0f, cheapest)\n', scenarioNames{wCost}, costUSD(wCost));
fprintf('\n  NOTE: C3''s cost includes an added automatic-controller estimate\n');
fprintf('  ($%.0f, industry price-range midpoint -- see CONFIG comment). This is\n', C3_controllerCostUSD);
fprintf('  NOT a formal vendor quote for this project; replace with a real quote\n');
fprintf('  before treating this cost comparison as final.\n');

resultsTbl = table(scenarioNames, complianceRate, avgPFmargin, avgLossMW, faultMarginVals, nameplateMvar, costUSD, scores, ...
    'VariableNames', {'Scenario','ComplianceRate','AvgPFmargin','AvgLoss_MW','FaultMargin','NameplateMvar','EstCostUSD','Score'});
writetable(resultsTbl, 'decision_support_hourly_results.csv');
fprintf('\nFull results written to decision_support_hourly_results.csv\n');

%% ===================== SECTION 5: CHARTS ============================
fprintf('\n=== Section 5: generating comparison charts ===\n');

figure('Position',[100 100 700 450]);
bar(categorical(scenarioNames), scores, 'FaceColor',[0.30 0.45 0.69]);
ylim([0 1]); ylabel('Score (0-1)'); title('Overall Weighted Ranking Score');
grid on; saveas(gcf, 'chart_overall_score.png');

figure('Position',[100 100 700 450]);
bar(categorical(scenarioNames), 100*complianceRate, 'FaceColor',[0.33 0.66 0.41]);
ylim([0 100]); ylabel('% of hours passing PF & voltage targets');
title('Hourly Compliance Rate (4 test hours)'); grid on;
saveas(gcf, 'chart_compliance_rate.png');

figure('Position',[100 100 700 450]);
bar(categorical(scenarioNames), avgLossMW, 'FaceColor',[0.77 0.31 0.32]);
ylabel('MW'); title('Average Feeder Losses Across 4 Hours'); grid on;
saveas(gcf, 'chart_avg_losses.png');

figure('Position',[100 100 700 450]);
bar(categorical(scenarioNames), costUSD, 'FaceColor',[0.51 0.45 0.70]);
ylabel('USD'); title('Estimated Installed Cost (bank + control hardware)'); grid on;
saveas(gcf, 'chart_cost.png');

figure('Position',[100 100 800 500]);
hold on;
markers = {'-o','-s','-^'};
for i = 1:length(scenarioNames)
    name = scenarioNames{i};
    rows = hourlyTbl(strcmp(hourlyTbl.Scenario, name), :);
    [sortedH, ord] = sort(rows.Hour);
    plot(sortedH, rows.PF(ord), markers{mod(i-1,3)+1}, 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', name);
end
yline(0.95, '--k', 'PF Target = 0.95', 'LabelHorizontalAlignment','left');
xlabel('Hour'); ylabel('Power Factor'); ylim([0 1.05]);
title('Power Factor Across the 4 Test Hours'); legend('Location','southeast');
grid on; hold off;
saveas(gcf, 'chart_pf_by_hour.png');

fprintf('5 chart PNGs saved: chart_overall_score.png, chart_compliance_rate.png,\n');
fprintf('chart_avg_losses.png, chart_cost.png, chart_pf_by_hour.png\n');

%% ================= SECTION 6: WRITE FULL REPORT =====================
% Everything printed above, captured permanently in one file -- console
% output disappears when MATLAB closes; this doesn't. HTML so it opens
% directly by double-clicking (any web browser), with real formatted
% tables and the charts embedded inline -- no separate viewer or app
% needed, unlike a .md file.
reportFile = 'decision_support_report.html';
fid = fopen(reportFile, 'w');
if fid == -1
    error('decision_support_ranking_hourly:reportWriteFailed', 'Could not open %s for writing.', reportFile);
end

fprintf(fid, '<!DOCTYPE html><html><head><meta charset="utf-8">\n');
fprintf(fid, '<title>Capacitor Design Decision-Support Report</title>\n');
fprintf(fid, ['<style>body{font-family:Arial,Helvetica,sans-serif;max-width:900px;margin:30px auto;' ...
    'padding:0 20px;color:#222;} h1{border-bottom:3px solid #333;padding-bottom:8px;} ' ...
    'h2{border-bottom:1px solid #ccc;padding-bottom:4px;margin-top:40px;} ' ...
    'table{border-collapse:collapse;width:100%%;margin:12px 0;} ' ...
    'th,td{border:1px solid #ccc;padding:6px 10px;text-align:left;} ' ...
    'th{background:#f0f0f0;} tr:nth-child(even){background:#fafafa;} ' ...
    '.pass{color:#1a7a1a;font-weight:bold;} .fail{color:#c0392b;font-weight:bold;} ' ...
    '.suggest{color:#8a6d00;font-style:italic;font-size:0.9em;} ' ...
    '.winner{background:#e8f5e9;font-weight:bold;} ' ...
    'img{max-width:100%%;border:1px solid #ddd;margin:10px 0;} ' ...
    '.note{background:#fff8e1;border-left:4px solid #f0c040;padding:8px 12px;margin:10px 0;font-size:0.9em;}' ...
    '</style></head><body>\n']);

fprintf(fid, '<h1>Capacitor Design Decision-Support Report</h1>\n');
fprintf(fid, '<p>Generated by decision_support_ranking_hourly.m</p>\n');

fprintf(fid, '<h2>1. Hourly Compliance (PF &ge; %.2f, voltage in [%.2f, %.2f] pu)</h2>\n', PF_target, Vband(1), Vband(2));
for i = 1:length(scenarioNames)
    name = scenarioNames{i};
    rows = hourlyTbl(strcmp(hourlyTbl.Scenario, name), :);
    fprintf(fid, '<h3>%s</h3>\n<table><tr><th>Hour</th><th>PF</th><th>Min V (pu)</th><th>Max V (pu)</th><th>Result</th></tr>\n', name);
    for r = 1:height(rows)
        pf = rows.PF(r); minV = rows.MinV_pu(r); maxV = rows.MaxV_pu(r); h = rows.Hour(r);
        passPF = pf >= PF_target;
        passV = (minV >= Vband(1)) && (maxV <= Vband(2));
        passAll = passPF && passV;
        cls = tern(passAll, 'pass', 'fail');
        fprintf(fid, '<tr><td>%d</td><td>%.4f</td><td>%.4f</td><td>%.4f</td><td class="%s">%s</td></tr>\n', ...
            h, pf, minV, maxV, cls, tern(passAll,'PASS','FAIL'));
        if ~passPF
            fprintf(fid, '<tr><td colspan="5" class="suggest">Suggested: Resize Capacitor, or Split Across Buses</td></tr>\n');
        end
        if minV < Vband(1)
            fprintf(fid, '<tr><td colspan="5" class="suggest">Suggested: Relocate Capacitor, or Resize Capacitor (undervoltage)</td></tr>\n');
        end
        if maxV > Vband(2)
            fprintf(fid, '<tr><td colspan="5" class="suggest">Suggested: Reduce Capacitor Size, or Add Switching (overvoltage)</td></tr>\n');
        end
    end
    fprintf(fid, '</table><p><b>%d/%d hours pass (%.0f%%)</b></p>\n', ...
        sum((rows.PF>=PF_target)&(rows.MinV_pu>=Vband(1))&(rows.MaxV_pu<=Vband(2))), height(rows), 100*complianceRate(i));
end

fprintf(fid, '<h2>2. Hour-Averaged Losses</h2>\n<table><tr><th>Scenario</th><th>Avg Loss (MW)</th></tr>\n');
for i = 1:length(scenarioNames)
    fprintf(fid, '<tr><td>%s</td><td>%.4f</td></tr>\n', scenarioNames{i}, avgLossMW(i));
end
fprintf(fid, '</table>\n');

fprintf(fid, '<h2>3. Fault Margin and Cost</h2>\n<table><tr><th>Scenario</th><th>Fault Margin</th><th>Nameplate (Mvar)</th><th>Cost (USD)</th></tr>\n');
for i = 1:length(scenarioNames)
    fprintf(fid, '<tr><td>%s</td><td>%.1f%%</td><td>%.4f</td><td>$%.0f</td></tr>\n', ...
        scenarioNames{i}, 100*faultMarginVals(i), nameplateMvar(i), costUSD(i));
end
fprintf(fid, ['</table><div class="note">C3 cost includes an added automatic-controller estimate ' ...
    '($%.0f, industry price-range midpoint). This is NOT a formal vendor quote for this project.</div>\n'], C3_controllerCostUSD);

fprintf(fid, '<h2>4. Final Ranking</h2>\n');
fprintf(fid, '<p>Weights: compliance=%.2f, PF margin=%.2f, losses=%.2f, fault=%.2f, cost=%.2f</p>\n', ...
    weights.complianceRate, weights.avgPFmargin, weights.losses, weights.faultMargin, weights.cost);
fprintf(fid, '<table><tr><th>Scenario</th><th>Compliance</th><th>Avg PF Margin</th><th>Avg Loss (MW)</th><th>Fault Margin</th><th>Cost</th><th>Score</th></tr>\n');
for i = 1:length(scenarioNames)
    rowcls = tern(i==bestIdx, ' class="winner"', '');
    fprintf(fid, '<tr%s><td>%s</td><td>%.0f%%</td><td>%+.4f</td><td>%.4f</td><td>%.1f%%</td><td>$%.0f</td><td>%.4f</td></tr>\n', ...
        rowcls, scenarioNames{i}, 100*complianceRate(i), avgPFmargin(i), avgLossMW(i), 100*faultMarginVals(i), costUSD(i), scores(i));
end
fprintf(fid, '</table>\n<p><b>Highest-ranked scenario: %s (score = %.4f)</b></p>\n', scenarioNames{bestIdx}, bestScore);
fprintf(fid, '<p><i>This ranking is advisory only -- the engineer makes the final design call.</i></p>\n');

fprintf(fid, '<h2>5. Per-Factor Winners</h2>\n<table><tr><th>#</th><th>Factor</th><th>Winner</th></tr>\n');
fprintf(fid, '<tr><td>1</td><td>Compliance rate</td><td>%s (%.0f%%)</td></tr>\n', scenarioNames{wComp}, 100*complianceRate(wComp));
fprintf(fid, '<tr><td>2</td><td>Avg PF margin</td><td>%s (+%.4f)</td></tr>\n', scenarioNames{wPF}, avgPFmargin(wPF));
fprintf(fid, '<tr><td>3</td><td>Avg losses</td><td>%s (%.4f MW, lowest)</td></tr>\n', scenarioNames{wLoss}, avgLossMW(wLoss));
fprintf(fid, '<tr><td>4</td><td>Fault margin</td><td>%s (%.1f%%)</td></tr>\n', scenarioNames{wFault}, 100*faultMarginVals(wFault));
fprintf(fid, '<tr><td>5</td><td>Cost</td><td>%s ($%.0f, cheapest)</td></tr>\n', scenarioNames{wCost}, costUSD(wCost));
fprintf(fid, '</table>\n');

fprintf(fid, '<h2>Charts</h2>\n');
fprintf(fid, '<img src="chart_overall_score.png" alt="Overall Score"><br>\n');
fprintf(fid, '<img src="chart_compliance_rate.png" alt="Compliance Rate"><br>\n');
fprintf(fid, '<img src="chart_avg_losses.png" alt="Average Losses"><br>\n');
fprintf(fid, '<img src="chart_cost.png" alt="Cost"><br>\n');
fprintf(fid, '<img src="chart_pf_by_hour.png" alt="PF by Hour"><br>\n');

fprintf(fid, '</body></html>\n');
fclose(fid);
fprintf('\nFull written report saved to %s -- double-click to open in your browser.\n', reportFile);

%% ============================ HELPERS ==============================
function q = find_continuous_q(topo, bus, target, qmax)
    lo = 0.0; hi = qmax;
    for it = 1:50
        mid = (lo+hi)/2;
        m = containers.Map('KeyType','double','ValueType','double');
        m(bus) = mid;
        [~, ~, ~, Vmag] = bfs_solve_v2droop(topo, m);
        v = Vmag(topo.busIdxMap(bus));
        if v < target, lo = mid; else, hi = mid; end
    end
    q = hi;
end

function s = tern(cond, a, b)
    if cond, s = a; else, s = b; end
end

function n = norm01(v)
    lo = min(v); hi = max(v);
    if hi - lo < 1e-12, n = ones(size(v));
    else, n = (v - lo) / (hi - lo); end
end
