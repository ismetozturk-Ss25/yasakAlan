function [az_next, el_next, status, debug_node] = avoidance_step_ml( ...
    az_now, el_now, az_cmd, el_cmd, ...
    env_az_min, env_el_min, env_az_max, env_el_max, ...
    forbidden, az_wrap, motion_profile, ...
    graph_nc, graph_naz, graph_nel, graph_adj)
%AVOIDANCE_STEP_ML  Forbidden-zone avoidance command shaper (MATLAB version)
%
%  Drop-in replacement for sfun_avoidance_opmode.c
%  Use inside a Simulink MATLAB Function block.
%
%  Algorithm (5 steps):
%    1. Escape if inside forbidden zone
%    2. Clamp + project target
%    3. Direct path clear? -> OK_DIRECT
%    4. A* path (compute or follow cached)
%    5. No path -> hold position
%
%  INPUTS:
%    az_now, el_now          - current turret position  (single scalar)
%    az_cmd, el_cmd          - commanded target          (single scalar)
%    env_az_min/el_min/az_max/el_max - envelope          (single scalar)
%    forbidden [32x5]        - [valid, az_min, el_min, az_max, el_max]
%    az_wrap                 - 0 or 1
%    motion_profile          - 0=LINEAR, 1=AZ_THEN_EL, 2=EL_THEN_AZ
%    graph_nc                - node count from PreOp     (int16)
%    graph_naz [1x256]       - node AZ coords            (single)
%    graph_nel [1x256]       - node EL coords            (single)
%    graph_adj [1x8192]      - adjacency bytes           (uint8)
%
%  OUTPUTS:
%    az_next, el_next        - next command  (single)
%    status                  - 0=DIRECT, 1=WAYPOINT, 2=NO_PATH  (int16)
%    debug_node              - target node index (-1=direct, -2=no_path)

% ---- Constants (must match C code) ----
NFZ                     = int32(32);
CORNER_EPS              = single(0.1);
WP_REACH                = single(0.3);
MAX_PATH_LEN            = int32(48);
MAX_NODES               = int32(256);
ADJ_BYTES_C             = int32(32);
MAX_STEP                = single(0.0);
TGT_MOVE_THRESH         = single(2.0);

OK_DIRECT   = int16(0);
OK_WAYPOINT = int16(1);
NO_PATH_S   = int16(2);

% ---- Persistent state ----
persistent s_path_az s_path_el s_path_len s_path_idx s_path_valid
persistent s_path_tgt_az s_path_tgt_el

if isempty(s_path_valid)
    s_path_az  = zeros(1, MAX_PATH_LEN, 'single');
    s_path_el  = zeros(1, MAX_PATH_LEN, 'single');
    s_path_len = int32(0);  s_path_idx = int32(1);
    s_path_valid  = int32(0);
    s_path_tgt_az = single(0);  s_path_tgt_el = single(0);
end

% ---- Cast inputs ----
cur_az = single(az_now);
cur_el = single(el_now);
wrap   = int32(az_wrap);
prof   = int32(motion_profile);
nc     = int32(graph_nc);
eAZmin = single(env_az_min);
eELmin = single(env_el_min);
eAZmax = single(env_az_max);
eELmax = single(env_el_max);

% ---- Parse forbidden zones into separate arrays ----
fv  = zeros(1, NFZ, 'single');   % valid
fa1 = zeros(1, NFZ, 'single');   % az_min
fe1 = zeros(1, NFZ, 'single');   % el_min
fa2 = zeros(1, NFZ, 'single');   % az_max
fe2 = zeros(1, NFZ, 'single');   % el_max
for k = 1:NFZ
    fv(k)  = single(forbidden(k,1));
    fa1(k) = single(forbidden(k,2));
    fe1(k) = single(forbidden(k,3));
    fa2(k) = single(forbidden(k,4));
    fe2(k) = single(forbidden(k,5));
end

% ---- Split wrapped zones ----
[fv, fa1, fe1, fa2, fe2] = split_wrapped(fv, fa1, fe1, fa2, fe2, NFZ, eAZmin, eAZmax);

% ---- Graph data ----
g_naz = single(graph_naz);
g_nel = single(graph_nel);
g_adj = uint8(graph_adj);

% ==================================================================
%  STEP 1: Escape if inside forbidden zone
% ==================================================================
[escaped, esc_az, esc_el] = try_escape_fn(cur_az, cur_el, ...
    fv, fa1, fe1, fa2, fe2, eAZmin, eELmin, eAZmax, eELmax, wrap, NFZ, CORNER_EPS);
if escaped
    az_next = esc_az;  el_next = esc_el;  status = OK_WAYPOINT;
    s_path_valid = int32(0);
    debug_node = single(find_nearest(esc_az, esc_el, g_naz, g_nel, nc, wrap));
    return;
end

% ==================================================================
%  STEP 2: Clamp + project target
% ==================================================================
tgt_az = clamp_val(single(az_cmd), eAZmin, eAZmax);
tgt_el = clamp_val(single(el_cmd), eELmin, eELmax);
if wrap ~= int32(0), tgt_az = norm_az(tgt_az); end

[tgt_az, tgt_el] = project_target_fn(tgt_az, tgt_el, ...
    fv, fa1, fe1, fa2, fe2, eAZmin, eELmin, eAZmax, eELmax, wrap, NFZ, CORNER_EPS);

% ==================================================================
%  STEP 3: Direct path clear?
% ==================================================================
if is_path_clear(cur_az, cur_el, tgt_az, tgt_el, fv, fa1, fe1, fa2, fe2, prof, wrap, NFZ)
    az_next = tgt_az;  el_next = tgt_el;  status = OK_DIRECT;
    s_path_valid = int32(0);
    debug_node = single(-1);
    [az_next, el_next] = rate_limit(az_next, el_next, cur_az, cur_el, ...
                                     wrap, MAX_STEP, eAZmin, eELmin, eAZmax, eELmax);
    return;
end

% ==================================================================
%  STEP 4: A* path
% ==================================================================

% 4a. Invalidate cache if target moved
if s_path_valid ~= int32(0)
    if manhattan(tgt_az, tgt_el, s_path_tgt_az, s_path_tgt_el, wrap) > TGT_MOVE_THRESH
        s_path_valid = int32(0);
    end
end

% 4b. Compute new A* path if no valid cache
if s_path_valid == int32(0)
    src = find_nearest_reachable(cur_az, cur_el, g_naz, g_nel, nc, wrap, ...
                                  fv, fa1, fe1, fa2, fe2, prof, NFZ);
    dst = find_nearest_reachable(tgt_az, tgt_el, g_naz, g_nel, nc, wrap, ...
                                  fv, fa1, fe1, fa2, fe2, prof, NFZ);
    [plen, paz, pel] = astar(src, dst, g_naz, g_nel, g_adj, ...
                              nc, wrap, MAX_PATH_LEN, ADJ_BYTES_C, MAX_NODES);
    if plen > int32(0)
        s_path_len = plen;  s_path_idx = int32(1);
        s_path_valid = int32(1);
        for pi = 1:plen
            s_path_az(pi) = paz(pi);
            s_path_el(pi) = pel(pi);
        end
        s_path_tgt_az = tgt_az;  s_path_tgt_el = tgt_el;
    end
end

% 4c. Follow cached path
if s_path_valid ~= int32(0) && s_path_idx <= s_path_len
    wp_az = s_path_az(s_path_idx);
    wp_el = s_path_el(s_path_idx);

    if manhattan(cur_az, cur_el, wp_az, wp_el, wrap) < WP_REACH
        s_path_idx = s_path_idx + 1;
        if s_path_idx > s_path_len
            s_path_valid = int32(0);
        else
            wp_az = s_path_az(s_path_idx);
            wp_el = s_path_el(s_path_idx);
        end
    end

    if s_path_valid ~= int32(0) && s_path_idx <= s_path_len
        % Shortcut: try direct to target
        if is_path_clear(cur_az, cur_el, tgt_az, tgt_el, fv, fa1, fe1, fa2, fe2, prof, wrap, NFZ)
            az_next = tgt_az;  el_next = tgt_el;  status = OK_DIRECT;
            s_path_valid = int32(0);
            debug_node = single(-1);
            [az_next, el_next] = rate_limit(az_next, el_next, cur_az, cur_el, ...
                                             wrap, MAX_STEP, eAZmin, eELmin, eAZmax, eELmax);
            return;
        end

        if pt_in_env(wp_az, wp_el, eAZmin, eELmin, eAZmax, eELmax) && ...
           ~pt_in_any_fz(wp_az, wp_el, fv, fa1, fe1, fa2, fe2, NFZ) && ...
           is_path_clear(cur_az, cur_el, wp_az, wp_el, fv, fa1, fe1, fa2, fe2, prof, wrap, NFZ)
            az_next = wp_az;  el_next = wp_el;  status = OK_WAYPOINT;
            debug_node = single(find_nearest(wp_az, wp_el, g_naz, g_nel, nc, wrap));
            [az_next, el_next] = rate_limit(az_next, el_next, cur_az, cur_el, ...
                                             wrap, MAX_STEP, eAZmin, eELmin, eAZmax, eELmax);
            return;
        end
        s_path_valid = int32(0);
    end
end

% Retry A* if path just got invalidated
if s_path_valid == int32(0)
    src = find_nearest_reachable(cur_az, cur_el, g_naz, g_nel, nc, wrap, ...
                                  fv, fa1, fe1, fa2, fe2, prof, NFZ);
    dst = find_nearest_reachable(tgt_az, tgt_el, g_naz, g_nel, nc, wrap, ...
                                  fv, fa1, fe1, fa2, fe2, prof, NFZ);
    [plen, paz, pel] = astar(src, dst, g_naz, g_nel, g_adj, ...
                              nc, wrap, MAX_PATH_LEN, ADJ_BYTES_C, MAX_NODES);
    if plen > int32(0)
        s_path_len = plen;  s_path_idx = int32(1);
        s_path_valid = int32(1);
        for pi = 1:plen
            s_path_az(pi) = paz(pi);
            s_path_el(pi) = pel(pi);
        end
        s_path_tgt_az = tgt_az;  s_path_tgt_el = tgt_el;

        az_next = s_path_az(1);  el_next = s_path_el(1);
        status  = OK_WAYPOINT;
        debug_node = single(find_nearest(az_next, el_next, g_naz, g_nel, nc, wrap));
        [az_next, el_next] = rate_limit(az_next, el_next, cur_az, cur_el, ...
                                         wrap, MAX_STEP, eAZmin, eELmin, eAZmax, eELmax);
        return;
    end
end

% ==================================================================
%  STEP 5: No path
% ==================================================================
az_next = cur_az;  el_next = cur_el;  status = NO_PATH_S;
s_path_valid = int32(0);
debug_node = single(-2);

end  % avoidance_step_ml


% =====================================================================
%  LOCAL FUNCTIONS  (all use plain arrays, no structs -- Coder safe)
% =====================================================================

function v = clamp_val(v, lo, hi)
    if v < lo, v = lo; end
    if v > hi, v = hi; end
end

function az = norm_az(az)
    if az > single(180),   az = az - single(360); end
    if az <= single(-180), az = az + single(360); end
end

function d = manhattan(ax, ay, bx, by, wrap)
    daz = abs(ax - bx);
    if wrap ~= int32(0) && daz > single(180), daz = single(360) - daz; end
    d = daz + abs(ay - by);
end

function yes = pt_in_rect_sc(az, el, r_az_min, r_el_min, r_az_max, r_el_max)
    yes = (az > r_az_min && az < r_az_max && ...
           el > r_el_min && el < r_el_max);
end

function yes = pt_in_env(az, el, eAZmin, eELmin, eAZmax, eELmax)
    yes = (az >= eAZmin && az <= eAZmax && ...
           el >= eELmin && el <= eELmax);
end

function yes = pt_in_any_fz(az, el, fv, fa1, fe1, fa2, fe2, nfz)
    yes = false;
    for i = 1:nfz
        if fv(i) == single(0), continue; end
        if pt_in_rect_sc(az, el, fa1(i), fe1(i), fa2(i), fe2(i))
            yes = true; return;
        end
    end
end

% ---- Split wrapped zones (az_min > az_max) ----
function [fv, fa1, fe1, fa2, fe2] = split_wrapped(fv, fa1, fe1, fa2, fe2, nfz, eAZmin, eAZmax)
    for k = 1:nfz
        if fv(k) == single(0), continue; end
        if fa1(k) <= fa2(k), continue; end
        orig_az_max = fa2(k);
        fa2(k) = eAZmax;
        for s = 1:nfz
            if fv(s) == single(0)
                fv(s)  = single(1);
                fa1(s) = eAZmin;
                fe1(s) = fe1(k);
                fa2(s) = orig_az_max;
                fe2(s) = fe2(k);
                break;
            end
        end
    end
end

% ---- Liang-Barsky segment vs AABB ----
function hit = seg_hits_rect(p0x, p0y, p1x, p1y, r_az_min, r_el_min, r_az_max, r_el_max)
    dx = p1x - p0x;
    dy = p1y - p0y;
    t_near = single(0);  t_far = single(1);

    if abs(dx) < single(1e-9)
        if p0x <= r_az_min || p0x >= r_az_max, hit = false; return; end
    else
        inv_d = single(1.0) / dx;
        t1 = (r_az_min - p0x) * inv_d;
        t2 = (r_az_max - p0x) * inv_d;
        if t1 > t2, tmp = t1; t1 = t2; t2 = tmp; end
        t_near = max(t_near, t1);
        t_far  = min(t_far,  t2);
        if t_near >= t_far, hit = false; return; end
    end

    if abs(dy) < single(1e-9)
        if p0y <= r_el_min || p0y >= r_el_max, hit = false; return; end
    else
        inv_d = single(1.0) / dy;
        t1 = (r_el_min - p0y) * inv_d;
        t2 = (r_el_max - p0y) * inv_d;
        if t1 > t2, tmp = t1; t1 = t2; t2 = tmp; end
        t_near = max(t_near, t1);
        t_far  = min(t_far,  t2);
        if t_near >= t_far, hit = false; return; end
    end

    hit = true;
end

% ---- Segment clear (wrap-aware) ----
function clr = seg_is_clear(p0x, p0y, p1x, p1y, fv, fa1, fe1, fa2, fe2, wrap, nfz)
    daz = p1x - p0x;
    if wrap ~= int32(0) && (daz > single(180) || daz < single(-180))
        if daz < single(-180)
            eff_daz = daz + single(360);  bnd = single(180);
        else
            eff_daz = daz - single(360);  bnd = single(-180);
        end
        if abs(eff_daz) < single(1e-9), clr = true; return; end
        t_split  = (bnd - p0x) / eff_daz;
        el_split = p0y + t_split * (p1y - p0y);
        for ii = 1:nfz
            if fv(ii) == single(0), continue; end
            if seg_hits_rect(p0x, p0y, bnd, el_split, fa1(ii), fe1(ii), fa2(ii), fe2(ii))
                clr = false; return;
            end
        end
        for ii = 1:nfz
            if fv(ii) == single(0), continue; end
            if seg_hits_rect(-bnd, el_split, p1x, p1y, fa1(ii), fe1(ii), fa2(ii), fe2(ii))
                clr = false; return;
            end
        end
        clr = true; return;
    end

    for ii = 1:nfz
        if fv(ii) == single(0), continue; end
        if seg_hits_rect(p0x, p0y, p1x, p1y, fa1(ii), fe1(ii), fa2(ii), fe2(ii))
            clr = false; return;
        end
    end
    clr = true;
end

% ---- Path clear (motion profile aware) ----
function clr = is_path_clear(p0az, p0el, p1az, p1el, fv, fa1, fe1, fa2, fe2, prof, wrap, nfz)
    if prof == int32(1)  % AZ_THEN_EL
        if ~seg_is_clear(p0az, p0el, p1az, p0el, fv, fa1, fe1, fa2, fe2, wrap, nfz)
            clr = false; return;
        end
        clr = seg_is_clear(p1az, p0el, p1az, p1el, fv, fa1, fe1, fa2, fe2, wrap, nfz);
    elseif prof == int32(2)  % EL_THEN_AZ
        if ~seg_is_clear(p0az, p0el, p0az, p1el, fv, fa1, fe1, fa2, fe2, wrap, nfz)
            clr = false; return;
        end
        clr = seg_is_clear(p0az, p1el, p1az, p1el, fv, fa1, fe1, fa2, fe2, wrap, nfz);
    else  % LINEAR (0) -- only check diagonal
        clr = seg_is_clear(p0az, p0el, p1az, p1el, fv, fa1, fe1, fa2, fe2, wrap, nfz);
    end
end

% ---- Project target out of forbidden zone ----
function [taz, tel] = project_target_fn(taz, tel, ...
    fv, fa1, fe1, fa2, fe2, eAZmin, eELmin, eAZmax, eELmax, wrap, nfz, eps)
    for i = 1:nfz
        if fv(i) == single(0), continue; end
        if ~pt_in_rect_sc(taz, tel, fa1(i), fe1(i), fa2(i), fe2(i)), continue; end

        d1 = abs(taz - fa1(i));
        d2 = abs(taz - fa2(i));
        d3 = abs(tel - fe1(i));
        d4 = abs(tel - fe2(i));

        p_az1 = fa1(i) - eps;  p_el1 = tel;
        p_az2 = fa2(i) + eps;  p_el2 = tel;
        p_az3 = taz;           p_el3 = fe1(i) - eps;
        p_az4 = taz;           p_el4 = fe2(i) + eps;

        % Find closest boundary
        best = int32(1);  best_d = d1;
        if d2 < best_d, best = int32(2); best_d = d2; end
        if d3 < best_d, best = int32(3); best_d = d3; end %#ok<NASGU>
        if d4 < best_d, best = int32(4); end

        if best == int32(1)
            taz = clamp_val(p_az1, eAZmin, eAZmax);
            tel = clamp_val(p_el1, eELmin, eELmax);
        elseif best == int32(2)
            taz = clamp_val(p_az2, eAZmin, eAZmax);
            tel = clamp_val(p_el2, eELmin, eELmax);
        elseif best == int32(3)
            taz = clamp_val(p_az3, eAZmin, eAZmax);
            tel = clamp_val(p_el3, eELmin, eELmax);
        else
            taz = clamp_val(p_az4, eAZmin, eAZmax);
            tel = clamp_val(p_el4, eELmin, eELmax);
        end
        if wrap ~= int32(0), taz = norm_az(taz); end
        break;
    end
end

% ---- Try escape ----
function [escaped, out_az, out_el] = try_escape_fn(cur_az, cur_el, ...
    fv, fa1, fe1, fa2, fe2, eAZmin, eELmin, eAZmax, eELmax, wrap, nfz, eps)
    escaped = false;  out_az = cur_az;  out_el = cur_el;
    for i = 1:nfz
        if fv(i) == single(0), continue; end
        if ~pt_in_rect_sc(cur_az, cur_el, fa1(i), fe1(i), fa2(i), fe2(i)), continue; end

        cand_az = [fa1(i) - eps, fa2(i) + eps, cur_az,         cur_az];
        cand_el = [cur_el,       cur_el,       fe1(i) - eps, fe2(i) + eps];
        cand_d  = [abs(cur_az - fa1(i)), abs(cur_az - fa2(i)), ...
                   abs(cur_el - fe1(i)), abs(cur_el - fe2(i))];

        [~, order] = sort(cand_d);
        for j = 1:4
            idx = order(j);
            az_c = clamp_val(cand_az(idx), eAZmin, eAZmax);
            el_c = clamp_val(cand_el(idx), eELmin, eELmax);
            if wrap ~= int32(0), az_c = norm_az(az_c); end
            if ~pt_in_env(az_c, el_c, eAZmin, eELmin, eAZmax, eELmax), continue; end
            if pt_in_any_fz(az_c, el_c, fv, fa1, fe1, fa2, fe2, nfz), continue; end
            out_az = az_c;  out_el = el_c;  escaped = true;  return;
        end
        return;
    end
end

% ---- Nearest graph node (by distance only, for debug) ----
function idx = find_nearest(az, el, naz, nel, nc, wrap)
    best_d = single(1e30);  idx = int32(-1);
    for i = 1:nc
        d = manhattan(az, el, naz(i), nel(i), wrap);
        if d < best_d
            best_d = d;  idx = int32(i - 1);  % 0-based for debug
        end
    end
end

% ---- Nearest reachable graph node ----
function idx = find_nearest_reachable(az, el, naz, nel, nc, wrap, ...
    fv, fa1, fe1, fa2, fe2, prof, nfz)
    best_reach_d = single(1e30);  best_reach = int32(-1);
    best_any_d   = single(1e30);  best_any   = int32(-1);
    for i = 1:nc
        d = manhattan(az, el, naz(i), nel(i), wrap);
        if d < best_any_d
            best_any_d = d;  best_any = int32(i - 1);
        end
        if is_path_clear(az, el, naz(i), nel(i), fv, fa1, fe1, fa2, fe2, prof, wrap, nfz)
            if d < best_reach_d
                best_reach_d = d;  best_reach = int32(i - 1);
            end
        end
    end
    if best_reach >= int32(0)
        idx = best_reach;
    else
        idx = best_any;
    end
end

% ---- Adjacency bit read (0-based node indices) ----
function yes = adj_get(adj_flat, i0, j0, adj_bytes)
    byte_idx = int32(i0) * adj_bytes + int32(floor(double(j0) / 8.0)) + int32(1);
    bit_idx  = int32(mod(j0, int32(8)));
    yes = bitand(uint16(adj_flat(byte_idx)), bitshift(uint16(1), bit_idx)) ~= uint16(0);
end

% ---- A* on precomputed graph ----
function [plen, out_az, out_el] = astar(src0, dst0, naz, nel, adj_flat, nc, wrap, ...
                                         max_path, adj_bytes, max_nodes)
    out_az = zeros(1, max_path, 'single');
    out_el = zeros(1, max_path, 'single');
    plen = int32(0);

    if src0 < int32(0) || dst0 < int32(0) || src0 == dst0, return; end

    INF_COST = single(1e30);
    g_cost = INF_COST * ones(1, max_nodes, 'single');
    f_cost = INF_COST * ones(1, max_nodes, 'single');
    par    = -ones(1, max_nodes, 'int32');
    closed = false(1, max_nodes);

    src = int32(src0 + 1);  dst = int32(dst0 + 1);

    goal_az = naz(dst);  goal_el = nel(dst);
    g_cost(src) = single(0);
    f_cost(src) = manhattan(naz(src), nel(src), goal_az, goal_el, wrap);

    cur = int32(-1);
    for iter = 1:nc %#ok<FORFLG>
        cur = int32(-1);  best_f = INF_COST;
        for ii = 1:nc
            if ~closed(ii) && f_cost(ii) < best_f
                best_f = f_cost(ii);  cur = int32(ii);
            end
        end
        if cur < int32(0), return; end
        if cur == dst, break; end
        closed(cur) = true;

        for jj = 1:nc
            if closed(jj) || jj == cur, continue; end
            if ~adj_get(adj_flat, int32(cur-1), int32(jj-1), adj_bytes), continue; end
            edge_c = manhattan(naz(cur), nel(cur), naz(jj), nel(jj), wrap);
            tent_g = g_cost(cur) + edge_c;
            if tent_g < g_cost(jj)
                g_cost(jj) = tent_g;
                f_cost(jj) = tent_g + manhattan(naz(jj), nel(jj), goal_az, goal_el, wrap);
                par(jj) = cur;
            end
        end
    end

    if cur ~= dst, return; end

    % Reconstruct path (exclude src, include dst)
    total = int32(0);
    node = dst;
    while node ~= src
        total = total + int32(1);
        node = par(node);
        if node < int32(1) || total > nc, plen = int32(0); return; end
    end

    plen_val = min(total, max_path);

    node = dst;
    for skip = 1:(total - plen_val) %#ok<FORFLG>
        node = par(node);
    end
    for ii = plen_val:-1:int32(1)
        out_az(ii) = naz(node);
        out_el(ii) = nel(node);
        node = par(node);
    end
    plen = plen_val;
end

% ---- Rate limiter ----
function [az, el] = rate_limit(az, el, cur_az, cur_el, wrap, max_step, eAZmin, eELmin, eAZmax, eELmax)
    if max_step <= single(0), return; end
    daz = az - cur_az;
    del_v = el - cur_el;
    if wrap ~= int32(0)
        if daz > single(180),  daz = daz - single(360); end
        if daz < single(-180), daz = daz + single(360); end
    end
    dist = abs(daz) + abs(del_v);
    if dist > max_step
        scale = max_step / dist;
        az = cur_az + daz * scale;
        el = cur_el + del_v * scale;
        if wrap ~= int32(0), az = norm_az(az); end
        az = clamp_val(az, eAZmin, eAZmax);
        el = clamp_val(el, eELmin, eELmax);
    end
end
