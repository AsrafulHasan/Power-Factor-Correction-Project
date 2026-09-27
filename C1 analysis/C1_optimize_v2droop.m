%% C1_optimize_v2droop.m
%
%  PURPOSE: Find the genuine minimum-nameplate SINGLE-BANK capacitor
%  placement (the "C1" scenario) using real V^2 droop capacitor physics
%  and a full voltage-band constraint (PF >= target AND every bus
%  voltage in [Vmin,Vmax]).
%
%  IMPORTANT CORRECTNESS NOTE: the feasible region for a single bus's
%  nameplate Mvar is NOT "everything above some threshold" -- it is a
%  BOUNDED BAND. Too little capacitance fails PF / undervoltage; too
%  MUCH capacitance overshoots the voltage ceiling (Vmax_limit) just as
%  easily. A naive bisection between 0 and a fixed ceiling breaks if
%  that ceiling happens to sit past the feasible band (it did, for
%  every bus on this feeder, at a 5 Mvar ceiling) -- it would wrongly
%  report the bus as infeasible even though a real feasible band exists
%  at a smaller size. This version fixes that: it SCANS upward from 0
%  in fine steps to find the first point where feasibility begins (the
%  transition from infeasible to feasible), then bisects within that
%  narrow bracket to refine the exact boundary. This is robust
%  regardless of whether the feasible region is a threshold or a band.
%
%  Uses bfs_topology_build.m + bfs_solve_v2droop.m (fast, no-file-I/O
%  solver pair). Searches every load bus by default, not a pre-selected
%  shortlist -- see CONFIG to restrict this if some buses are not
%  physically viable for a capacitor installation.

clear; clc;

%% ============================= CONFIG =============================
branchCSV   = 'branch_data.csv';
loadCSV     = 'load_data_peak.csv';
slackBus    = 650;
baseMVA     = 5.0;
PF_target   = 0.95;
Vmin_limit  = 0.95;      % pu
Vmax_limit  = 1.05;      % pu
searchCeilingMvar = 5.0; % upper bound of the scan -- must be comfortably
                          % above where you'd ever expect a feasible
                          % band to sit for this feeder
scanSteps   = 400;        % resolution of the coarse scan; must be fine
                          % enough not to step over a narrow feasible
                          % band entirely (step size = ceiling/scanSteps)

restrictToBuses = [];   % leave empty to search every load bus; or e.g. [671,675,692] to restrict

%% ======================= INPUT VALIDATION ==========================
if ~isfile(branchCSV)
    error('C1_optimize_v2droop:fileNotFound', 'Branch data file not found: %s', branchCSV);
end
if ~isfile(loadCSV)
    error('C1_optimize_v2droop:fileNotFound', 'Load data file not found: %s', loadCSV);
end

loadTbl = readtable(loadCSV);
requiredCols = {'Bus','P_MW','Q_Mvar'};
missing = setdiff(requiredCols, loadTbl.Properties.VariableNames);
if ~isempty(missing)
    error('C1_optimize_v2droop:missingColumns', ...
        '%s is missing required column(s): %s', loadCSV, strjoin(missing, ', '));
end

candidateBuses = unique(loadTbl.Bus(loadTbl.P_MW > 0 | loadTbl.Q_Mvar > 0))';
if ~isempty(restrictToBuses)
    badBuses = setdiff(restrictToBuses, candidateBuses);
    if ~isempty(badBuses)
        error('C1_optimize_v2droop:badRestriction', ...
            'restrictToBuses contains bus(es) with no load in %s: [%s]', loadCSV, num2str(badBuses));
    end
    candidateBuses = restrictToBuses;
end

if isempty(candidateBuses)
    error('C1_optimize_v2droop:noCandidates', 'No candidate buses found -- check %s.', loadCSV);
end

tic;
topo = bfs_topology_build(branchCSV, loadCSV, slackBus, baseMVA);
fprintf('Topology built in %.3f sec. Searching %d candidate bus(es): %s\n\n', ...
    toc, length(candidateBuses), num2str(candidateBuses));

badRefs = setdiff(candidateBuses, topo.allBuses);
if ~isempty(badRefs)
    error('C1_optimize_v2droop:unknownBus', ...
        'Candidate bus(es) [%s] do not exist in the feeder topology (%s).', num2str(badRefs), branchCSV);
end

%% ========================== RUN SEARCH =============================
fprintf('=== Single-bank placement search (droop-aware, full voltage band) ===\n');
fprintf('(feasibility = PF >= %.2f AND every bus voltage in [%.2f, %.2f] pu)\n\n', ...
    PF_target, Vmin_limit, Vmax_limit);

tic;
results = struct('bus',{},'nameplate',{},'feasible',{},'PF',{},'minV',{},'maxV',{},'actualQ',{});
for i = 1:length(candidateBuses)
    b = candidateBuses(i);
    [nameplate, isFeasible] = find_feasible_boundary_single(topo, PF_target, Vmin_limit, Vmax_limit, ...
        b, searchCeilingMvar, scanSteps);

    if isFeasible
        m = containers.Map('KeyType','double','ValueType','double');
        m(b) = nameplate;
        [pf, ~, ~, Vmag, Qact] = bfs_solve_v2droop(topo, m);
        actualQ = Qact(topo.busIdxMap(b));
        fprintf('  Bus %-4d : nameplate = %.4f Mvar (actual injected %.4f) -> PF=%.4f, minV=%.4f, maxV=%.4f\n', ...
            b, nameplate, actualQ, pf, min(Vmag), max(Vmag));
        results(end+1) = struct('bus',b,'nameplate',nameplate,'feasible',true,'PF',pf, ...
            'minV',min(Vmag),'maxV',max(Vmag),'actualQ',actualQ); %#ok<AGROW>
    else
        fprintf('  Bus %-4d : NOT feasible anywhere in [0, %.1f] Mvar (scanned at %.4f Mvar resolution)\n', ...
            b, searchCeilingMvar, searchCeilingMvar/scanSteps);
        results(end+1) = struct('bus',b,'nameplate',NaN,'feasible',false,'PF',NaN, ...
            'minV',NaN,'maxV',NaN,'actualQ',NaN); %#ok<AGROW>
    end
end
fprintf('\nSearch completed in %.2f sec.\n', toc);

%% ========================== BEST RESULT ============================
feasibleResults = results([results.feasible]);
if isempty(feasibleResults)
    error('C1_optimize_v2droop:noFeasibleBus', ...
        'No candidate bus reached feasibility anywhere in [0, %.1f] Mvar. Increase searchCeilingMvar, increase scanSteps, or check your PF/voltage targets.', searchCeilingMvar);
end

[bestNameplate, bestIdx] = min([feasibleResults.nameplate]);
best = feasibleResults(bestIdx);

fprintf('\n>> BEST SINGLE-BANK (C1) RECOMMENDATION:\n');
fprintf('   Bus %d : %.4f Mvar nameplate (actual injected %.4f Mvar at converged voltage)\n', ...
    best.bus, best.nameplate, best.actualQ);
fprintf('   Verified: PF = %.4f, minV = %.4f pu, maxV = %.4f pu\n\n', best.PF, best.minV, best.maxV);
fprintf('   Write this to your capacitor config:\n');
fprintf('   Bus,Mvar\n   %d,%.4f\n', best.bus, best.nameplate);

%% ========================= EXPORT RESULTS ==========================
busCol = [results.bus]';
nameplateCol = [results.nameplate]';
feasibleCol = [results.feasible]';
PFCol = [results.PF]';
minVCol = [results.minV]';
maxVCol = [results.maxV]';
actualQCol = [results.actualQ]';

resultsTbl = table(busCol, nameplateCol, feasibleCol, PFCol, minVCol, maxVCol, actualQCol, ...
    'VariableNames', {'Bus','NameplateMvar','Feasible','PF','MinV_pu','MaxV_pu','ActualInjectedMvar'});
writetable(resultsTbl, 'C1_optimize_results.csv');
fprintf('Full results written to C1_optimize_results.csv\n');

%% ============================ HELPERS ==============================
function [nameplate, feasible] = find_feasible_boundary_single(topo, PF_target, Vmin_limit, Vmax_limit, ...
        bus, ceilingMvar, scanSteps)
% Scans upward from 0 to find the first infeasible->feasible transition
% (the lower edge of the feasible band), then bisects within that
% narrow bracket to refine it. Does NOT assume the region above some
% point is always feasible -- correct for bounded feasible bands.
    step = ceilingMvar / scanSteps;
    prevQ = 0; prevFeasible = false;
    nameplate = NaN; feasible = false;
    for k = 1:scanSteps
        q = k*step;
        m = containers.Map('KeyType','double','ValueType','double');
        m(bus) = q;
        f = is_feasible_droop(topo, m, PF_target, Vmin_limit, Vmax_limit);
        if f && ~prevFeasible
            lo = prevQ; hi = q;
            for it = 1:60
                mid = (lo+hi)/2;
                m2 = containers.Map('KeyType','double','ValueType','double');
                m2(bus) = mid;
                if is_feasible_droop(topo, m2, PF_target, Vmin_limit, Vmax_limit)
                    hi = mid;
                else
                    lo = mid;
                end
            end
            nameplate = hi; feasible = true;
            return;
        end
        prevQ = q; prevFeasible = f;
    end
end

function ok = is_feasible_droop(topo, capMap, PF_target, Vmin_limit, Vmax_limit)
    [pf, ~, ~, Vmag] = bfs_solve_v2droop(topo, capMap);
    ok = (pf >= PF_target) && all(Vmag >= Vmin_limit) && all(Vmag <= Vmax_limit);
end
