%% C2_optimize_v2droop.m  (fast version)
%
%  Same search as before (single-bus + pair placements, droop-aware,
%  full PF+voltage-band constraint, real-catalog-cap Stage C) but the
%  feeder topology and load data are read and built ONCE via
%  bfs_topology_build.m, then reused for every one of the ~19,000
%  solver calls the search makes via bfs_solve_v2droop.m. The previous
%  version re-read both CSVs and rebuilt the tree structure on every
%  single call, which is why it was slow -- this version should run in
%  a few seconds instead of minutes, with IDENTICAL results (same
%  electrical model, just no redundant file I/O).

clear; clc;

%% ---- Settings ----
branchCSV  = 'branch_data.csv';
loadCSV    = 'load_data_peak.csv';
slackBus   = 650;
baseMVA    = 5.0;
PF_target  = 0.95;
Vmin_limit = 0.95;
Vmax_limit = 1.05;
shortlist  = [634, 645, 646, 671, 675, 692];

tic;
topo = bfs_topology_build(branchCSV, loadCSV, slackBus, baseMVA);
fprintf('Topology built once in %.3f sec.\n\n', toc);

%% ---- STAGE A: single-bus placements ----
fprintf('=== STAGE A: single-bus placements (droop-aware) ===\n');
fprintf('(feasibility = PF >= %.2f AND every bus voltage in [%.2f, %.2f] pu)\n\n', ...
    PF_target, Vmin_limit, Vmax_limit);

tic;
singleResults = struct('bus',{},'nameplate',{},'PF',{},'minV',{},'maxV',{},'actualQ',{});
for i = 1:length(shortlist)
    b = shortlist(i);
    nameplate = min_nameplate_single(topo, PF_target, Vmin_limit, Vmax_limit, b);
    m = containers.Map('KeyType','double','ValueType','double');
    m(b) = nameplate;
    [pf, ~, ~, Vmag, Qact] = bfs_solve_v2droop(topo, m);
    actualQ = Qact(topo.busIdxMap(b));
    singleResults(end+1) = struct('bus',b,'nameplate',nameplate,'PF',pf, ...
        'minV',min(Vmag),'maxV',max(Vmag),'actualQ',actualQ); %#ok<AGROW>
    fprintf('  Bus %d alone : nameplate = %.4f Mvar (actual injected %.4f) -> PF=%.4f, minV=%.4f\n', ...
        b, nameplate, actualQ, pf, min(Vmag));
end
fprintf('Stage A done in %.2f sec.\n\n', toc);

%% ---- STAGE B: pair placements ----
fprintf('=== STAGE B: pair placements (droop-aware) ===\n\n');
tic;
pairs = nchoosek(shortlist, 2);
pairResults = struct('b1',{},'b2',{},'nameplate',{},'r',{},'PF',{},'minV',{},'maxV',{});
for k = 1:size(pairs,1)
    b1 = pairs(k,1); b2 = pairs(k,2);
    bestTotal = inf; bestR = 0.5;
    for r = 0:0.05:1
        t = min_nameplate_pair(topo, PF_target, Vmin_limit, Vmax_limit, b1, b2, r);
        if t < bestTotal
            bestTotal = t; bestR = r;
        end
    end
    m = containers.Map('KeyType','double','ValueType','double');
    m(b1) = bestTotal*bestR; m(b2) = bestTotal*(1-bestR);
    [pf, ~, ~, Vmag] = bfs_solve_v2droop(topo, m);
    pairResults(end+1) = struct('b1',b1,'b2',b2,'nameplate',bestTotal,'r',bestR, ...
        'PF',pf,'minV',min(Vmag),'maxV',max(Vmag)); %#ok<AGROW>
    fprintf('  Bus %d + Bus %d : nameplate = %.4f Mvar (split %.0f:%.0f) -> PF=%.4f, minV=%.4f\n', ...
        b1, b2, bestTotal, bestR*100, (1-bestR)*100, pf, min(Vmag));
end
fprintf('Stage B done in %.2f sec.\n\n', toc);

%% ---- OVERALL COMPARISON ----
fprintf('=== OVERALL COMPARISON (droop-aware, lowest nameplate wins) ===\n');
[~, siMin] = min([singleResults.nameplate]);
[~, paMin] = min([pairResults.nameplate]);
bestSingle = singleResults(siMin);
bestPair   = pairResults(paMin);

fprintf('Best single-bus : Bus %d, %.4f Mvar nameplate, PF=%.4f, minV=%.4f\n', ...
    bestSingle.bus, bestSingle.nameplate, bestSingle.PF, bestSingle.minV);
fprintf('Best pair (uncapped) : Bus %d + Bus %d, %.4f Mvar nameplate total, PF=%.4f, minV=%.4f\n\n', ...
    bestPair.b1, bestPair.b2, bestPair.nameplate, bestPair.PF, bestPair.minV);

%% ---- STAGE C: real catalog cap (1.2 Mvar/bank) ----
maxCatalogBankMvar = 1.2;
fprintf('=== STAGE C: enforce a REAL catalog cap of %.2f Mvar/bank ===\n', maxCatalogBankMvar);
if bestSingle.nameplate <= maxCatalogBankMvar
    fprintf('The best single bank (%.4f Mvar at Bus %d) already fits a %.2f Mvar catalog unit.\n', ...
        bestSingle.nameplate, bestSingle.bus, maxCatalogBankMvar);
    fprintf('   Bus,Mvar\n   %d,%.4f\n', bestSingle.bus, bestSingle.nameplate);
else
    fprintf('The best single bank (%.4f Mvar at Bus %d) EXCEEDS %.2f Mvar -- searching genuine 2-bank splits:\n\n', ...
        bestSingle.nameplate, bestSingle.bus, maxCatalogBankMvar);

    bestCapped = struct('b1',0,'b2',0,'total',inf,'q1',0,'q2',0);
    tic;
    for k = 1:size(pairs,1)
        b1 = pairs(k,1); b2 = pairs(k,2);
        [total, q1, q2] = min_total_pair_capped(topo, PF_target, Vmin_limit, Vmax_limit, b1, b2, maxCatalogBankMvar);
        if ~isinf(total)
            fprintf('  Bus %d + Bus %d : total = %.4f  (Bus %d = %.4f, Bus %d = %.4f)\n', ...
                b1, b2, total, b1, q1, b2, q2);
            if total < bestCapped.total
                bestCapped = struct('b1',b1,'b2',b2,'total',total,'q1',q1,'q2',q2);
            end
        end
    end
    fprintf('Stage C done in %.2f sec.\n', toc);

    fprintf('\n>> FINAL 2-BANK RECOMMENDATION (catalog-cap enforced, genuinely searched):\n');
    fprintf('   Bus %d : %.4f Mvar\n', bestCapped.b1, bestCapped.q1);
    fprintf('   Bus %d : %.4f Mvar\n', bestCapped.b2, bestCapped.q2);
    m = containers.Map('KeyType','double','ValueType','double');
    m(bestCapped.b1) = bestCapped.q1; m(bestCapped.b2) = bestCapped.q2;
    [pfF, ~, ~, VmagF] = bfs_solve_v2droop(topo, m);
    fprintf('   Verified: PF = %.4f, minV = %.4f, maxV = %.4f\n\n', pfF, min(VmagF), max(VmagF));
    fprintf('   Write this to capacitor_config.csv:\n');
    fprintf('   Bus,Mvar\n   %d,%.4f\n   %d,%.4f\n', bestCapped.b1, bestCapped.q1, bestCapped.b2, bestCapped.q2);
end

%% ================= LOCAL FUNCTIONS =================

function ok = is_feasible_droop(topo, capMap, PF_target, Vmin_limit, Vmax_limit)
    [pf, ~, ~, Vmag] = bfs_solve_v2droop(topo, capMap);
    ok = (pf >= PF_target) && all(Vmag >= Vmin_limit) && all(Vmag <= Vmax_limit);
end

function nameplate = min_nameplate_single(topo, PF_target, Vmin_limit, Vmax_limit, bus)
    lo = 0.0; hi = 4.0;
    for it = 1:60
        mid = (lo+hi)/2;
        m = containers.Map('KeyType','double','ValueType','double');
        m(bus) = mid;
        if is_feasible_droop(topo, m, PF_target, Vmin_limit, Vmax_limit)
            hi = mid;
        else
            lo = mid;
        end
    end
    nameplate = hi;
end

function total = min_nameplate_pair(topo, PF_target, Vmin_limit, Vmax_limit, b1, b2, r)
    lo = 0.0; hi = 4.0;
    for it = 1:60
        mid = (lo+hi)/2;
        m = containers.Map('KeyType','double','ValueType','double');
        m(b1) = mid*r; m(b2) = mid*(1-r);
        if is_feasible_droop(topo, m, PF_target, Vmin_limit, Vmax_limit)
            hi = mid;
        else
            lo = mid;
        end
    end
    total = hi;
end

function [total, q1, q2] = min_total_pair_capped(topo, PF_target, Vmin_limit, Vmax_limit, b1, b2, maxBank)
    bestTotal = inf; bestQ1 = 0; bestQ2 = 0;
    for r = 0:0.025:1
        lo = 0.0; hi = 4.0;
        for it = 1:60
            mid = (lo+hi)/2;
            q1 = min(mid*r, maxBank);
            q2 = min(mid*(1-r), maxBank);
            m = containers.Map('KeyType','double','ValueType','double');
            m(b1) = q1; m(b2) = q2;
            if is_feasible_droop(topo, m, PF_target, Vmin_limit, Vmax_limit)
                hi = mid;
            else
                lo = mid;
            end
        end
        q1 = min(hi*r, maxBank);
        q2 = min(hi*(1-r), maxBank);
        if q1 <= maxBank + 1e-9 && q2 <= maxBank + 1e-9 && hi < bestTotal
            bestTotal = hi; bestQ1 = q1; bestQ2 = q2;
        end
    end
    total = bestTotal; q1 = bestQ1; q2 = bestQ2;
end
