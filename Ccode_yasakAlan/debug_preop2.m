%% debug_preop2.m -- Print graph from Simulink workspace variables
%
% Reads graph_nc, graph_naz, graph_nel, graph_adj from workspace
% (output of the PreOp S-function block) and prints nodes + adjacency.
%
% Usage:
%   1. Run Simulink model so PreOp outputs land in workspace
%   2. Run this script

%% ============================================================
%  CONSTANTS (must match avoidance_simulink.h)
%  ============================================================
AVD_MAX_FIXED_NODES = 256;
AVD_ADJ_BYTES       = 32;

%% ============================================================
%  READ FROM WORKSPACE
%  ============================================================
nc  = double(graph_nc.data(1,:));
naz = double(graph_naz.data(1,:));
nel = double(graph_nel.data(1,:));

% graph_adj is uint8 [1x8192], row-major flattened adj[256][32]
adj_flat = uint8(graph_adj.data(1,:));
adj = reshape(adj_flat, AVD_ADJ_BYTES, AVD_MAX_FIXED_NODES)';  % [256 x 32]

%% ============================================================
%  PRINT NODES
%  ============================================================
fprintf('============================================================\n');
fprintf('  Graph: %d nodes  (from Simulink workspace)\n', nc);
fprintf('============================================================\n');
for i = 1:nc
    fprintf('  Node %3d: AZ=%7.1f  EL=%5.1f\n', i-1, naz(i), nel(i));
end

%% ============================================================
%  COUNT EDGES
%  ============================================================
edge_count = 0;
for i = 1:nc
    for j = (i+1):nc
        byte_idx = floor((j-1)/8) + 1;
        bit_idx  = mod(j-1, 8);
        if bitand(adj(i, byte_idx), uint8(bitshift(uint8(1), bit_idx))) ~= 0
            edge_count = edge_count + 1;
        end
    end
end
fprintf('\n  %d edges\n', edge_count);

%% ============================================================
%  ADJACENCY LIST
%  ============================================================
fprintf('\n============================================================\n');
fprintf('  ADJACENCY LIST\n');
fprintf('============================================================\n');
for i = 1:nc
    neighbors = [];
    for j = 1:nc
        byte_idx = floor((j-1)/8) + 1;
        bit_idx  = mod(j-1, 8);
        if bitand(adj(i, byte_idx), uint8(bitshift(uint8(1), bit_idx))) ~= 0
            neighbors(end+1) = j-1; %#ok<AGROW>
        end
    end
    fprintf('  Node %2d (%7.1f,%5.1f) -> [%s ]\n', ...
        i-1, naz(i), nel(i), sprintf(' %d', neighbors));
end

fprintf('\n>> Done. %d nodes, %d edges.\n', nc, edge_count);
