function truth = generateTruth3D(cfg)
% generateTruth3D
% ------------------------------------------------------------
% 3D UAM truth trajectory generator
%
% Flight sequence:
% TAKEOFF -> TOP_HOVER -> CRUISE/TURN/APPROACH -> DESCENT -> BOTTOM_HOVER
%
% Outputs:
%   truth.pos       : [East North Altitude]
%   truth.vel       : [vx vy vz]
%   truth.vh        : horizontal speed
%   truth.vtot      : total speed
%   truth.phase     : phase label
%   truth.phaseTau  : normalized progress in each contiguous phase block
%   truth.keyPoints : P1~P5 labels for plotting
% ------------------------------------------------------------

dt = cfg.dt;
cruiseAlt = cfg.cruise_altitude_m;

%% Default options
if ~isfield(cfg, 'enable_turn_phase')
    cfg.enable_turn_phase = true;
end

if ~isfield(cfg, 'enable_approach_phase')
    cfg.enable_approach_phase = true;
end

if ~isfield(cfg, 'turn_region_radius_m')
    cfg.turn_region_radius_m = 250;
end

%% 1) 2D corridor path from waypoints
raw2d = [];

for k = 1:size(cfg.uam_waypoints,1)-1
    p0 = cfg.uam_waypoints(k,:);
    p1 = cfg.uam_waypoints(k+1,:);

    nseg = max(2, ceil(norm(p1-p0) / cfg.path_resample_m));
    seg = [linspace(p0(1), p1(1), nseg)', ...
           linspace(p0(2), p1(2), nseg)'];

    if isempty(raw2d)
        raw2d = seg;
    else
        raw2d = [raw2d; seg(2:end,:)]; %#ok<AGROW>
    end
end

path2d = raw2d;

%% 2) Smooth 2D corridor
win = cfg.path_smooth_window;

if mod(win,2) == 0
    win = win + 1;
end

win = min(win, size(path2d,1)-1);

if mod(win,2) == 0
    win = win - 1;
end

if win >= 5
    path2d(:,1) = smoothdata(path2d(:,1), 'sgolay', win);
    path2d(:,2) = smoothdata(path2d(:,2), 'sgolay', win);
end

%% 3) Cumulative path distance
ds = [0; vecnorm(diff(path2d),2,2)];
s = cumsum(ds);
totalDist = s(end);

%% 4) Vertical takeoff
N_take = round(cfg.takeoff_time_s / dt);
tau_take = linspace(0,1,N_take)';

z_take = cruiseAlt * smoothstep(tau_take);
xy_take = repmat(path2d(1,:), N_take, 1);

P_take = [xy_take, z_take];

%% 5) Cruise / turn / final approach speed profile
vMax = cfg.max_cruise_speed;
aMax = cfg.max_cruise_accel;

s_now = 0;
v_now = 0;

s_list = [];
v_cmd_list = [];
brake_flag_list = [];

while s_now < totalDist
    remain = totalDist - s_now;
    brakeDist = v_now^2 / (2*aMax + eps);

    is_braking = remain <= brakeDist;

    if is_braking
        v_next = max(v_now - aMax*dt, 0);
    else
        v_next = min(v_now + aMax*dt, vMax);
    end

    s_now = min(s_now + 0.5*(v_now + v_next)*dt, totalDist);
    v_now = v_next;

    s_list = [s_list; s_now]; %#ok<AGROW>
    v_cmd_list = [v_cmd_list; v_now]; %#ok<AGROW>
    brake_flag_list = [brake_flag_list; is_braking]; %#ok<AGROW>

    if s_now >= totalDist
        break;
    end
end

x_cruise = interp1(s, path2d(:,1), s_list, 'linear', 'extrap');
y_cruise = interp1(s, path2d(:,2), s_list, 'linear', 'extrap');
z_cruise = cruiseAlt * ones(size(x_cruise));

P_cruise = [x_cruise, y_cruise, z_cruise];
nCruise = size(P_cruise,1);

%% 6) Vertical descent
N_desc = round(cfg.descent_time_s / dt);
tau_desc = linspace(0,1,N_desc)';

z_desc = cruiseAlt * (1 - smoothstep(tau_desc));
xy_desc = repmat(path2d(end,:), N_desc, 1);

P_desc = [xy_desc, z_desc];

%% 7) Hover phases
N_top_hover = round(cfg.top_hover_time_s / dt);
N_bottom_hover = round(cfg.bottom_hover_time_s / dt);

hover_top = repmat([path2d(1,:), cruiseAlt], N_top_hover, 1);
hover_bottom = repmat([path2d(end,:), 0], N_bottom_hover, 1);

%% 8) Combine full trajectory
pos = [P_take;
       hover_top;
       P_cruise;
       P_desc;
       hover_bottom];

N = size(pos,1);

%% 9) Initial phase labels
phase = strings(N,1);

idx_take_1 = 1;
idx_take_2 = size(P_take,1);

idx_top_1 = idx_take_2 + 1;
idx_top_2 = idx_top_1 + size(hover_top,1) - 1;

idx_cruise_1 = idx_top_2 + 1;
idx_cruise_2 = idx_cruise_1 + size(P_cruise,1) - 1;

idx_desc_1 = idx_cruise_2 + 1;
idx_desc_2 = idx_desc_1 + size(P_desc,1) - 1;

idx_bottom_1 = idx_desc_2 + 1;
idx_bottom_2 = N;

phase(idx_take_1:idx_take_2) = "TAKEOFF";

if N_top_hover > 0
    phase(idx_top_1:idx_top_2) = "TOP_HOVER";
end

phase(idx_cruise_1:idx_cruise_2) = "CRUISE";
phase(idx_desc_1:idx_desc_2) = "DESCENT";

if idx_bottom_1 <= idx_bottom_2
    phase(idx_bottom_1:idx_bottom_2) = "BOTTOM_HOVER";
end

%% 10) TURN phase: JP-1, JP-2, JP-3 주변
if cfg.enable_turn_phase
    cruise_xy = P_cruise(:,1:2);
    turn_mask = false(nCruise,1);

    for kk = 2:size(cfg.uam_waypoints,1)-1
        jp = cfg.uam_waypoints(kk,:);
        d_jp = vecnorm(cruise_xy - jp, 2, 2);
        turn_mask = turn_mask | (d_jp <= cfg.turn_region_radius_m);
    end

    idx_turn = idx_cruise_1 - 1 + find(turn_mask);
    phase(idx_turn) = "TURN";
end

%% 11) APPROACH phase: final braking only
% Important:
%   Do NOT label initial acceleration as APPROACH.
%   APPROACH means final deceleration before vertical descent.
if cfg.enable_approach_phase
    approach_mask = brake_flag_list(:);

    local_phase = phase(idx_cruise_1:idx_cruise_2);
    idx_app_local = find(approach_mask & (local_phase == "CRUISE"));

    idx_app = idx_cruise_1 - 1 + idx_app_local;
    phase(idx_app) = "APPROACH";
end

%% 12) Kinematics from position
vel = zeros(N,3);

if N >= 3
    vel(2:N-1,:) = (pos(3:N,:) - pos(1:N-2,:)) / (2*dt);
    vel(1,:) = (pos(2,:) - pos(1,:)) / dt;
    vel(N,:) = (pos(N,:) - pos(N-1,:)) / dt;
elseif N == 2
    vel(1,:) = (pos(2,:) - pos(1,:)) / dt;
    vel(2,:) = vel(1,:);
end

acc = zeros(N,3);

if N >= 3
    acc(2:N-1,:) = (vel(3:N,:) - vel(1:N-2,:)) / (2*dt);
end

vx = vel(:,1);
vy = vel(:,2);
vz = vel(:,3);

vh = sqrt(vx.^2 + vy.^2);
vtot = sqrt(vx.^2 + vy.^2 + vz.^2);

%% 13) phaseTau: normalized progress for each contiguous phase block
phaseTau = nan(N,1);

changeIdx = [1; find(phase(2:end) ~= phase(1:end-1)) + 1; N+1];

for b = 1:numel(changeIdx)-1
    ii = changeIdx(b):changeIdx(b+1)-1;

    if numel(ii) == 1
        phaseTau(ii) = 1;
    else
        phaseTau(ii) = linspace(0,1,numel(ii))';
    end
end

%% 14) Output structure
truth.pos = pos;
truth.vel = vel;
truth.acc = acc;

truth.vx = vx;
truth.vy = vy;
truth.vz = vz;
truth.vh = vh;
truth.vtot = vtot;
truth.alt = pos(:,3);

truth.phase = phase;
truth.phaseTau = phaseTau;

truth.N = N;
truth.t = (0:N-1)' * dt;
truth.path2d = path2d;

%% 15) Key points for plotting
% P1~P5 correspond to V3, JP1, JP2, JP3, V4.
wp = cfg.uam_waypoints;

truth.keyPoints.labels = {'P1','P2','P3','P4','P5'};
truth.keyPoints.names  = {'V3','JP1','JP2','JP3','V4'};
truth.keyPoints.xyz = [ ...
    wp(1,1), wp(1,2), 30;
    wp(2,1), wp(2,2), cruiseAlt + 20;
    wp(3,1), wp(3,2), cruiseAlt + 20;
    wp(4,1), wp(4,2), cruiseAlt + 20;
    wp(5,1), wp(5,2), 30];

truth.junctionPoints.labels = {'JP1','JP2','JP3'};
truth.junctionPoints.xyz = [ ...
    wp(2,1), wp(2,2), cruiseAlt + 35;
    wp(3,1), wp(3,2), cruiseAlt + 35;
    wp(4,1), wp(4,2), cruiseAlt + 35];

end