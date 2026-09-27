function topo = bfs_topology_build(branchCSV, loadCSV, slackBus, baseMVA)
% BFS_TOPOLOGY_BUILD  Reads the branch/load CSVs and builds the feeder
% topology ONCE, returning a struct that bfs_solve_v2droop.m (or a
% similar fast solve function) can reuse thousands of times without
% re-reading files or rebuilding the tree structure each call.
%
% This is a pure performance refactor -- the electrical model is
% identical to bfs_powerflow_v2droop.m / bfs_powerflow.m. Only the
% expensive one-time setup (file I/O, containers.Map construction, tree
% ordering) is separated from the part that actually changes between
% optimizer iterations (the capacitor placement).

    branchTbl = readtable(branchCSV);
    fromB = branchTbl.FromBus;
    toB   = branchTbl.ToBus;
    Rb    = branchTbl.R_pu;
    Xb    = branchTbl.X_pu;

    allBuses = unique([fromB; toB]);
    nBus = length(allBuses);
    busIdxMap = containers.Map(num2cell(allBuses), num2cell(1:nBus));

    nBranch = length(Rb);
    fromIdx = zeros(nBranch,1);
    toIdx   = zeros(nBranch,1);
    for k = 1:nBranch
        fromIdx(k) = busIdxMap(fromB(k));
        toIdx(k)   = busIdxMap(toB(k));
    end

    if ~isKey(busIdxMap, slackBus)
        error('bfs_topology_build:badSlack', 'slackBus %d not found in branch data.', slackBus);
    end
    slackIdx = busIdxMap(slackBus);

    parentOf = zeros(nBus,1);
    parentBranch = zeros(nBus,1);
    children = cell(nBus,1);
    for k = 1:nBranch
        parentOf(toIdx(k)) = fromIdx(k);
        parentBranch(toIdx(k)) = k;
        children{fromIdx(k)} = [children{fromIdx(k)}, toIdx(k)];
    end

    order = slackIdx;
    queue = slackIdx;
    while ~isempty(queue)
        node = queue(1); queue(1) = [];
        for c = children{node}
            order = [order, c]; %#ok<AGROW>
            queue = [queue, c]; %#ok<AGROW>
        end
    end

    Pload = zeros(nBus,1);
    Qload_base = zeros(nBus,1);
    if ~isempty(loadCSV)
        loadTbl = readtable(loadCSV);
        for k = 1:height(loadTbl)
            b = loadTbl.Bus(k);
            if isKey(busIdxMap, b)
                Pload(busIdxMap(b)) = loadTbl.P_MW(k)/baseMVA;
                Qload_base(busIdxMap(b)) = loadTbl.Q_Mvar(k)/baseMVA;
            end
        end
    end

    topo.allBuses = allBuses;
    topo.busIdxMap = busIdxMap;
    topo.nBus = nBus;
    topo.nBranch = nBranch;
    topo.Rb = Rb;
    topo.Xb = Xb;
    topo.order = order;
    topo.parentOf = parentOf;
    topo.parentBranch = parentBranch;
    topo.slackIdx = slackIdx;
    topo.Pload = Pload;
    topo.Qload_base = Qload_base;
    topo.baseMVA = baseMVA;
end
