function fig = plotRouteAndProfiles_korean_obstacles(truth, cfg, buildings_for_draw, mountains_for_physics)
% plotRouteAndProfiles_korean_obstacles
% ------------------------------------------------------------
% 논문용 Figure (a)
%   - 3D UAM 경로
%   - MLAT 수신기 배치
%   - 건물(직육면체)
%   - 산/지형(원통)
%   - waypoint 이름 제거
%   - 한글 범례 사용
% ------------------------------------------------------------

if ~isfield(truth,'pos'), error('truth.pos is required'); end
if ~isfield(truth,'phase'), error('truth.phase is required'); end
if ~isfield(cfg,'dt'), cfg.dt = 1.0; end

truth = ensureTruthKinematics_local(truth, cfg);

pos   = truth.pos;
phase = string(truth.phase(:));
t     = truth.t(:);
alt   = truth.alt(:);
vh    = truth.vh(:);
vz    = truth.vz(:);
vtot  = truth.vtot(:);

rxPos = cfg.mlat_sites_3d;
rxPos = rxPos(1:min(size(rxPos,1),11), :);

tCruise   = firstPhaseTime_local(truth, "CRUISE");
tApproach = firstPhaseTime_local(truth, "APPROACH");
tDescent  = firstPhaseTime_local(truth, "DESCENT");
tBottom   = firstPhaseTime_local(truth, "BOTTOM_HOVER");

fig = figure('Color','w','Position',[100 60 1700 900]);
tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

%% (a) 시뮬레이션 환경
ax1 = nexttile(tl,[2 1]);
hold(ax1,'on'); grid(ax1,'on'); box(ax1,'on');
view(ax1, 38, 24);

% -----------------------------
% 1. 장애물 필터링 (경로 근처만)
% -----------------------------
xmin = min(pos(:,1)) - 500;
xmax = max(pos(:,1)) + 500;
ymin = min(pos(:,2)) - 500;
ymax = max(pos(:,2)) + 500;

% 건물
if ~isempty(buildings_for_draw)
    B = buildings_for_draw;
    maskB = B.X >= xmin & B.X <= xmax & B.Y >= ymin & B.Y <= ymax;
    B = B(maskB,:);

    % 너무 많으면 일부만 표시
    maxDrawBuildings = 180;
    if height(B) > maxDrawBuildings
        idx = round(linspace(1, height(B), maxDrawBuildings));
        B = B(idx,:);
    end

    for i = 1:height(B)
        drawCuboidWH(ax1, B.X(i), B.Y(i), B.W(i), B.H(i), B.Z(i), ...
            [0.78 0.78 0.78], 0.18);
    end
end

% 산/지형
if ~isempty(mountains_for_physics)
    M = mountains_for_physics;
    maskM = M(:,1) >= xmin & M(:,1) <= xmax & M(:,2) >= ymin & M(:,2) <= ymax;
    M = M(maskM,:);

    for i = 1:size(M,1)
        drawCylinderObstacle(ax1, M(i,1), M(i,2), M(i,3), M(i,4), ...
            [0.55 0.42 0.28], 0.12);
    end
end

% -----------------------------
% 2. UAM 경로
% -----------------------------
segments = findPhaseSegments_local(phase);
for s = 1:numel(segments)
    idx = segments(s).idx;
    ph  = segments(s).name;
    c   = phaseColor_local(ph);

    plot3(ax1, pos(idx,1), pos(idx,2), pos(idx,3), '-', ...
        'Color', c, 'LineWidth', 3.0, 'HandleVisibility','off');
end

% -----------------------------
% 3. MLAT 수신기
% -----------------------------
for i = 1:size(rxPos,1)
    x = rxPos(i,1);
    y = rxPos(i,2);
    h = max(rxPos(i,3), 30);

    towerW = 45; towerD = 45;
    drawCuboidCenter(ax1, [x, y, h/2], [towerW, towerD, h], [0.05 0.05 0.05], 0.92);
    plot3(ax1, x, y, h, 'k^', 'MarkerFaceColor','k', 'MarkerSize',7, ...
        'HandleVisibility','off');
    text(ax1, x, y, h+15, sprintf('R%d',i), ...
        'FontSize',10, 'FontWeight','bold', ...
        'HorizontalAlignment','center', ...
        'BackgroundColor','w', 'Margin',1);
end

xlabel(ax1,'동쪽 [m]','FontSize',12);
ylabel(ax1,'북쪽 [m]','FontSize',12);
zlabel(ax1,'고도 [m]','FontSize',12);
title(ax1,'(a) 시뮬레이션 환경', ...
    'FontSize',14,'FontWeight','bold');
set(ax1,'FontSize',11);

allX = [pos(:,1); rxPos(:,1)];
allY = [pos(:,2); rxPos(:,2)];
xlim(ax1,[min(allX)-700, max(allX)+700]);
ylim(ax1,[min(allY)-700, max(allY)+700]);
zlim(ax1,[0, max(pos(:,3))+100]);
pbaspect(ax1,[1.2 1.0 0.75]);

% 범례용 dummy
hTake = plot3(ax1,nan,nan,nan,'-','Color',phaseColor_local("TAKEOFF"),'LineWidth',3);
hTop  = plot3(ax1,nan,nan,nan,'-','Color',phaseColor_local("TOP_HOVER"),'LineWidth',3);
hCru  = plot3(ax1,nan,nan,nan,'-','Color',phaseColor_local("CRUISE"),'LineWidth',3);
hTurn = plot3(ax1,nan,nan,nan,'-','Color',phaseColor_local("TURN"),'LineWidth',3);
hApp  = plot3(ax1,nan,nan,nan,'-','Color',phaseColor_local("APPROACH"),'LineWidth',3);
hDes  = plot3(ax1,nan,nan,nan,'-','Color',phaseColor_local("DESCENT"),'LineWidth',3);
hBot  = plot3(ax1,nan,nan,nan,'-','Color',phaseColor_local("BOTTOM_HOVER"),'LineWidth',3);
hMlat = plot3(ax1,nan,nan,nan,'k^','MarkerFaceColor','k','MarkerSize',7);

legend(ax1, [hTake hTop hCru hTurn hApp hDes hBot hMlat], ...
    {'이륙','상단 정지','순항','선회','접근','하강','하단 정지','MLAT 수신기'}, ...
    'Location','northwest','FontSize',10);
end

%% ========================================================================
function truth = ensureTruthKinematics_local(truth, cfg)
pos = truth.pos;
N = size(pos,1);

if isfield(truth,'t')
    t = truth.t(:);
else
    t = (0:N-1)'*cfg.dt;
end

if isfield(truth,'vel')
    vel = truth.vel;
else
    vel = zeros(N,3);
    if N >= 3
        vel(2:N-1,:) = (pos(3:N,:) - pos(1:N-2,:)) / (2*cfg.dt);
        vel(1,:) = (pos(2,:) - pos(1,:)) / cfg.dt;
        vel(N,:) = (pos(N,:) - pos(N-1,:)) / cfg.dt;
    end
end

vx = vel(:,1); vy = vel(:,2); vz = vel(:,3);

truth.t = t;
truth.vel = vel;
truth.alt = pos(:,3);
truth.vh = sqrt(vx.^2 + vy.^2);
truth.vtot = sqrt(vx.^2 + vy.^2 + vz.^2);
truth.vz = vz;
end

function segments = findPhaseSegments_local(phase)
phase = string(phase(:));
N = numel(phase);
segments = struct('name',{},'idx',{});

if N==0, return; end

iStart = 1; cnt = 0;
for k = 2:N
    if phase(k) ~= phase(k-1)
        cnt = cnt + 1;
        segments(cnt).name = phase(iStart);
        segments(cnt).idx  = iStart:(k-1);
        iStart = k;
    end
end
cnt = cnt + 1;
segments(cnt).name = phase(iStart);
segments(cnt).idx  = iStart:N;
end

function c = phaseColor_local(ph)
switch upper(string(ph))
    case "TAKEOFF",       c = [0.00 0.45 0.74];
    case "TOP_HOVER",     c = [0.85 0.33 0.10];
    case "CRUISE",        c = [0.93 0.69 0.13];
    case "TURN",          c = [0.49 0.18 0.56];
    case "APPROACH",      c = [0.47 0.67 0.19];
    case "DESCENT",       c = [0.30 0.75 0.93];
    case "BOTTOM_HOVER",  c = [0.64 0.08 0.18];
    otherwise,            c = [0.2 0.2 0.2];
end
end

function tFirst = firstPhaseTime_local(truth, phaseName)
idx = find(string(truth.phase)==string(phaseName),1,'first');
if isempty(idx), tFirst = nan;
else, tFirst = truth.t(idx);
end
end

function drawCuboidWH(ax, x, y, w, h, zTop, faceColor, faceAlpha)
cx = x; cy = y; cz = zTop/2;
drawCuboidCenter(ax, [cx cy cz], [w h zTop], faceColor, faceAlpha);
end

function drawCuboidCenter(ax, center, sizeXYZ, faceColor, faceAlpha)
cx = center(1); cy = center(2); cz = center(3);
dx = sizeXYZ(1)/2; dy = sizeXYZ(2)/2; dz = sizeXYZ(3)/2;

V = [cx-dx cy-dy cz-dz;
     cx+dx cy-dy cz-dz;
     cx+dx cy+dy cz-dz;
     cx-dx cy+dy cz-dz;
     cx-dx cy-dy cz+dz;
     cx+dx cy-dy cz+dz;
     cx+dx cy+dy cz+dz;
     cx-dx cy+dy cz+dz];

F = [1 2 3 4;
     5 6 7 8;
     1 2 6 5;
     2 3 7 6;
     3 4 8 7;
     4 1 5 8];

patch(ax,'Vertices',V,'Faces',F,...
    'FaceColor',faceColor,...
    'FaceAlpha',faceAlpha,...
    'EdgeColor',[0.6 0.6 0.6],...
    'LineWidth',0.2,...
    'HandleVisibility','off');
end

function drawCylinderObstacle(ax, x, y, r, h, faceColor, faceAlpha)
[nx, ny, nz] = cylinder(r, 18);
nz = nz * h;
surf(ax, nx + x, ny + y, nz, ...
    'FaceColor', faceColor, ...
    'FaceAlpha', faceAlpha, ...
    'EdgeColor', 'none', ...
    'HandleVisibility','off');
end