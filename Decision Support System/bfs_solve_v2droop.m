function [PF, Psrc_MW, Qsrc_Mvar, Vmag, Qcap_actual_MVar] = bfs_solve_v2droop(topo, capNominalMap)
% BFS_SOLVE_V2DROOP  Same electrical model as bfs_powerflow_v2droop.m
% (V^2 droop capacitor physics), but takes a PRE-BUILT topology struct
% (from bfs_topology_build.m) instead of file paths -- so no CSV
% re-reading or tree rebuilding happens here. This is the function an
% optimizer's inner loop should call; it is dramatically faster when
% called thousands of times, and gives numerically identical results to
% bfs_powerflow_v2droop.m for the same inputs.
%
% INPUTS:
%   topo           : struct from bfs_topology_build.m
%   capNominalMap  : containers.Map, keys = bus numbers, values =
%                     nameplate Mvar. Pass an empty containers.Map for
%                     no capacitors.

    Qnom_pu = zeros(topo.nBus,1);
    if ~isempty(capNominalMap)
        capBuses = keys(capNominalMap);
        for k = 1:length(capBuses)
            b = capBuses{k};
            if isKey(topo.busIdxMap, b)
                Qnom_pu(topo.busIdxMap(b)) = capNominalMap(b)/topo.baseMVA;
            end
        end
    end

    Vc = ones(topo.nBus,1) + 0i;
    Ibranch = zeros(topo.nBranch,1) + 0i;
    maxIter = 100;
    tol = 1e-8;

    for iter = 1:maxIter
        Vprev = Vc;

        Qcap_actual_pu = Qnom_pu .* (abs(Vc).^2);
        Qload = topo.Qload_base - Qcap_actual_pu;

        Ibus = zeros(topo.nBus,1) + 0i;
        for i = 1:topo.nBus
            if i == topo.slackIdx, continue; end
            Ibus(i) = conj((topo.Pload(i)+1i*Qload(i)) / Vc(i));
        end

        branchCurrentAtBus = Ibus;
        for idx = length(topo.order):-1:1
            b = topo.order(idx);
            if b == topo.slackIdx, continue; end
            br = topo.parentBranch(b);
            Ibranch(br) = branchCurrentAtBus(b);
            p = topo.parentOf(b);
            branchCurrentAtBus(p) = branchCurrentAtBus(p) + Ibranch(br);
        end

        for idx = 1:length(topo.order)
            b = topo.order(idx);
            if b == topo.slackIdx, continue; end
            br = topo.parentBranch(b);
            p = topo.parentOf(b);
            Vc(b) = Vc(p) - Ibranch(br)*(topo.Rb(br)+1i*topo.Xb(br));
        end

        if max(abs(Vc - Vprev)) < tol
            break;
        end
    end

    Qcap_actual_pu = Qnom_pu .* (abs(Vc).^2);
    Qload = topo.Qload_base - Qcap_actual_pu;

    totalLossP = sum(abs(Ibranch).^2 .* topo.Rb);
    totalLossQ = sum(abs(Ibranch).^2 .* topo.Xb);

    Psrc_MW   = (sum(topo.Pload) + totalLossP) * topo.baseMVA;
    Qsrc_Mvar = (sum(Qload) + totalLossQ) * topo.baseMVA;
    PF = Psrc_MW / sqrt(Psrc_MW^2 + Qsrc_Mvar^2);

    Vmag = abs(Vc);
    Qcap_actual_MVar = Qcap_actual_pu * topo.baseMVA;
end
