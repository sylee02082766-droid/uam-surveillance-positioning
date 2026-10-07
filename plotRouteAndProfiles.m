function fig = plotRouteAndProfiles(truth, cfg)
% plotRouteAndProfiles
% ------------------------------------------------------------
% 논문용 Figure 1 생성:
%   (a) 3D UAM route and fixed MLAT sites
%   (b-1) Altitude profile
%   (b-2) Velocity profile
%
% 주요 개선:
%   - MLAT는 cfg.mlat_sites_3d에서 11개만 사용
%   - MLAT를 검정 cuboid tower로 표시
%   - legend는 dummy handle로 직접 구성하여 data1, data2 방지
%   - phase는 연속 구간별로 plot하여 가짜 연결선 방지
%   - altitude profile에 CRUISE / APPROACH / DESCENT 시작 시간 표시
%   - velocity profile은 horizontal speed, vertical velocity,
%     speed magnitude를 분리 표시
% ------------------------------------------------------------

%% Basic checks
if ~isfield(truth, 'pos')
    error('truth.pos is required.');
end
if ~isfield(truth, 'phase')
    error('truth.phase is required.');
end
if ~isfield(cfg, 'dt')
    cfg.dt = 1.0;
end

%% Kinematics
truth = ensureTruthKinematics(truth, cfg);

pos = truth.pos;
phase = string(truth.phase(:));
t = truth.t(:);

alt = truth.alt(:);
vh = truth.vh(:);
vz = truth.vz(:);
vtot = truth.vtot(:);

%% MLAT receiver positions
if ~isfield(cfg, 'mlat_sites_3d')
    error('cfg.mlat_sites_3d is required to plot MLAT sites.');
end

rxPos = cfg.mlat_sites_3d;
rxPos = rxPos(1:min(11,size(rxPos,1)), :);  % 반드시 11개 이하만 그림

%% Key points
keyPoints = makeKeyPointsForPlot(truth, cfg);

%% Phase transition times
tCruise = getFirstPhaseTime(truth, "CRUISE");
tApproach = getFirstPhaseTime(truth, "APPROACH");
tDescent = getFirstPhaseTime(truth, "DESCENT");

%% Figure
fig = figure('Color','w','Position',[80 80 1700 900]);
tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

%% =========================================================
% (a) 3D UAM route and fixed MLAT sites
% ==========================================================
ax1 = nexttile(tl,[2 1]);
hold(ax1,'on');
grid(ax1,'on');
box(ax1,'on');

% 보기 각도
view(ax1, 38, 24);

% Phase route plotting by contiguous segment
displayPhase = sanitizeApproachForPlot(phase);
segments = findPhaseSegments(displayPhase);

for s = 1:numel(segments)
    idx = segments(s).idx;
    ph = char(segments(s).name);
    c = phaseColor(ph);

    plot3(ax1, pos(idx,1), pos(idx,2), pos(idx,3), '-', ...
        'Color', c, ...
        'LineWidth', 3.0, ...
        'HandleVisibility','off');
end

% MLAT tower 11개만 표시
for i = 1:size(rxPos,1)
    x = rxPos(i,1);
    y = rxPos(i,2);

    if size(rxPos,2) >= 3
        h = max(rxPos(i,3), 30);
    else
        h = 120;
    end

    % 실제 좌표 스케일이 크므로 논문 그림에서 잘 보이도록 45 m 폭 사용
    towerW = 45;
    towerD = 45;

    drawCuboidNoLegend(ax1, [x, y, h/2], [towerW, towerD, h], ...
        [0.05 0.05 0.05], 0.95);

    plot3(ax1, x, y, h, 'k^', ...
        'MarkerFaceColor','k', ...
        'MarkerSize',7, ...
        'HandleVisibility','off');

    text(ax1, x, y, h+15, sprintf('R%d',i), ...
        'FontSize',10, ...
        'FontWeight','bold', ...
        'HorizontalAlignment','center', ...
        'BackgroundColor','w', ...
        'Margin',1);
end

% Key points P1~P5
KP = keyPoints.xyz;
LB = keyPoints.labels;

for i = 1:size(KP,1)
    plot3(ax1, KP(i,1), KP(i,2), KP(i,3), 'ko', ...
        'MarkerFaceColor','y', ...
        'MarkerSize',7, ...
        'HandleVisibility','off');

    text(ax1, KP(i,1), KP(i,2), KP(i,3)+18, LB{i}, ...
        'FontSize',11, ...
        'FontWeight','bold', ...
        'HorizontalAlignment','center', ...
        'BackgroundColor','w', ...
        'Margin',1);
end

% Axis labels
xlabel(ax1,'East [m]','FontSize',12);
ylabel(ax1,'North [m]','FontSize',12);
zlabel(ax1,'Altitude [m]','FontSize',12);
title(ax1,'(a) 3D UAM route and fixed MLAT sites', ...
    'FontSize',14,'FontWeight','bold');

set(ax1,'FontSize',11);

% Axis limit
allX = [pos(:,1); rxPos(:,1)];
allY = [pos(:,2); rxPos(:,2)];

xMargin = 800;
yMargin = 800;

xlim(ax1, [min(allX)-xMargin, max(allX)+xMargin]);
ylim(ax1, [min(allY)-yMargin, max(allY)+yMargin]);

if isfield(cfg, 'cruise_altitude_m')
    zlim(ax1, [0, cfg.cruise_altitude_m + 80]);
else
    zlim(ax1, [0, max(pos(:,3))+80]);
end

% 축 비율: x-y가 너무 크므로 z가 잘 보이게 조정
pbaspect(ax1, [1.2 1.0 0.75]);

% Manual legend with dummy handles
hTake = plot3(ax1,nan,nan,nan,'-','Color',phaseColor("TAKEOFF"),'LineWidth',3);
hTop  = plot3(ax1,nan,nan,nan,'-','Color',phaseColor("TOP_HOVER"),'LineWidth',3);
hCru  = plot3(ax1,nan,nan,nan,'-','Color',phaseColor("CRUISE"),'LineWidth',3);
hTurn = plot3(ax1,nan,nan,nan,'-','Color',phaseColor("TURN"),'LineWidth',3);
hApp  = plot3(ax1,nan,nan,nan,'-','Color',phaseColor("APPROACH"),'LineWidth',3);
hDes  = plot3(ax1,nan,nan,nan,'-','Color',phaseColor("DESCENT"),'LineWidth',3);
hBot  = plot3(ax1,nan,nan,nan,'-','Color',phaseColor("BOTTOM_HOVER"),'LineWidth',3);
hMlat = plot3(ax1,nan,nan,nan,'k^','MarkerFaceColor','k','MarkerSize',7);

legend(ax1, ...
    [hTake hTop hCru hTurn hApp hDes hBot hMlat], ...
    {'TAKEOFF','TOP\_HOVER','CRUISE','TURN','APPROACH','DESCENT','BOTTOM\_HOVER','MLAT'}, ...
    'Location','northwest', ...
    'FontSize',10);

%% =========================================================
% (b-1) Altitude profile
% ==========================================================
ax2 = nexttile(tl,2);
hold(ax2,'on');
grid(ax2,'on');
box(ax2,'on');

plot(ax2, t, alt, 'k-', 'LineWidth',2.5);

xlabel(ax2,'Time [s]','FontSize',12);
ylabel(ax2,'Altitude [m]','FontSize',12);
title(ax2,'(b-1) Altitude profile','FontSize',13,'FontWeight','bold');
set(ax2,'FontSize',11);

% Transition annotations
yMaxAlt = max(alt);

if ~isnan(tCruise)
    xline(ax2, tCruise, '--', ...
        'Color', phaseColor("CRUISE"), ...
        'LineWidth',1.6, ...
        'HandleVisibility','off');

    text(ax2, tCruise+3, 0.92*yMaxAlt, ...
        sprintf('CRUISE start t = %d s', round(tCruise)), ...
        'Color', phaseColor("CRUISE"), ...
        'FontSize',10, ...
        'FontWeight','bold', ...
        'BackgroundColor','w');
end

if ~isnan(tApproach)
    xline(ax2, tApproach, ':', ...
        'Color', phaseColor("APPROACH"), ...
        'LineWidth',1.8, ...
        'HandleVisibility','off');

    text(ax2, tApproach+3, 0.82*yMaxAlt, ...
        sprintf('APPROACH start t = %d s', round(tApproach)), ...
        'Color', phaseColor("APPROACH"), ...
        'FontSize',10, ...
        'FontWeight','bold', ...
        'BackgroundColor','w');
end

if ~isnan(tDescent)
    xline(ax2, tDescent, '--', ...
        'Color', phaseColor("DESCENT"), ...
        'LineWidth',1.6, ...
        'HandleVisibility','off');

    text(ax2, tDescent+3, 0.68*yMaxAlt, ...
        sprintf('DESCENT start t = %d s', round(tDescent)), ...
        'Color', phaseColor("DESCENT"), ...
        'FontSize',10, ...
        'FontWeight','bold', ...
        'BackgroundColor','w');
end

%% =========================================================
% (b-2) Velocity profile
% ==========================================================
ax3 = nexttile(tl,4);
hold(ax3,'on');
grid(ax3,'on');
box(ax3,'on');

plot(ax3, t, vh, '-', ...
    'Color',[0.00 0.45 0.74], ...
    'LineWidth',2.2);

plot(ax3, t, vz, '--', ...
    'Color',[0.85 0.33 0.10], ...
    'LineWidth',2.2);

plot(ax3, t, vtot, '-.', ...
    'Color',[0.47 0.67 0.19], ...
    'LineWidth',2.2);

yline(ax3,0,'k:','HandleVisibility','off');

xlabel(ax3,'Time [s]','FontSize',12);
ylabel(ax3,'Velocity / speed [m/s]','FontSize',12);
title(ax3,'(b-2) Velocity profile','FontSize',13,'FontWeight','bold');

legend(ax3, ...
    {'Horizontal speed','Vertical velocity','Speed magnitude'}, ...
    'Location','best', ...
    'FontSize',10);

set(ax3,'FontSize',11);

% Same transition lines on velocity plot
if ~isnan(tCruise)
    xline(ax3, tCruise, '--', ...
        'Color', phaseColor("CRUISE"), ...
        'LineWidth',1.3, ...
        'HandleVisibility','off');
end

if ~isnan(tApproach)
    xline(ax3, tApproach, ':', ...
        'Color', phaseColor("APPROACH"), ...
        'LineWidth',1.5, ...
        'HandleVisibility','off');
end

if ~isnan(tDescent)
    xline(ax3, tDescent, '--', ...
        'Color', phaseColor("DESCENT"), ...
        'LineWidth',1.3, ...
        'HandleVisibility','off');
end

end

%% ========================================================================
% Helper functions
% ========================================================================

function truth = ensureTruthKinematics(truth, cfg)

pos = truth.pos;
N = size(pos,1);

if isfield(truth,'t')
    t = truth.t(:);
else
    t = (0:N-1)' * cfg.dt;
end

if isfield(truth,'vel')
    vel = truth.vel;
else
    vel = zeros(N,3);

    if N >= 3
        vel(2:N-1,:) = (pos(3:N,:) - pos(1:N-2,:)) / (2*cfg.dt);
        vel(1,:) = (pos(2,:) - pos(1,:)) / cfg.dt;
        vel(N,:) = (pos(N,:) - pos(N-1,:)) / cfg.dt;
    elseif N == 2
        vel(1,:) = (pos(2,:) - pos(1,:)) / cfg.dt;
        vel(2,:) = vel(1,:);
    end
end

vx = vel(:,1);
vy = vel(:,2);
vz = vel(:,3);

truth.t = t;
truth.vel = vel;
truth.vx = vx;
truth.vy = vy;
truth.vz = vz;
truth.vh = sqrt(vx.^2 + vy.^2);
truth.vtot = sqrt(vx.^2 + vy.^2 + vz.^2);
truth.alt = pos(:,3);

end

function keyPoints = makeKeyPointsForPlot(truth, cfg)

if isfield(truth,'keyPoints') && isfield(truth.keyPoints,'xyz') && isfield(truth.keyPoints,'labels')
    keyPoints = truth.keyPoints;
    return;
end

wp = cfg.uam_waypoints;

if isfield(cfg,'cruise_altitude_m')
    h = cfg.cruise_altitude_m;
else
    h = max(truth.pos(:,3));
end

keyPoints.labels = {'P1','P2','P3','P4','P5'};
keyPoints.xyz = [ ...
    wp(1,1), wp(1,2), 30;
    wp(2,1), wp(2,2), h+20;
    wp(3,1), wp(3,2), h+20;
    wp(4,1), wp(4,2), h+20;
    wp(5,1), wp(5,2), 30];

end

function displayPhase = sanitizeApproachForPlot(phase)
% APPROACH가 여러 조각으로 나뉘어 있으면 마지막 APPROACH만 유지.
% 초기 가속 구간이 APPROACH로 표시되는 문제를 방지하기 위한 plot용 처리.

displayPhase = string(phase(:));

segments = findPhaseSegments(displayPhase);
appSegIdx = [];

for i = 1:numel(segments)
    if segments(i).name == "APPROACH"
        appSegIdx(end+1) = i; %#ok<AGROW>
    end
end

if numel(appSegIdx) <= 1
    return;
end

% 마지막 APPROACH만 유지, 이전 APPROACH는 CRUISE로 표시
for k = 1:numel(appSegIdx)-1
    idx = segments(appSegIdx(k)).idx;
    displayPhase(idx) = "CRUISE";
end

end

function segments = findPhaseSegments(phase)

phase = string(phase(:));
N = numel(phase);

segments = struct('name',{},'idx',{});

if N == 0
    return;
end

iStart = 1;
count = 0;

for k = 2:N
    if phase(k) ~= phase(k-1)
        count = count + 1;
        segments(count).name = phase(iStart);
        segments(count).idx = iStart:(k-1);
        iStart = k;
    end
end

count = count + 1;
segments(count).name = phase(iStart);
segments(count).idx = iStart:N;

end

function c = phaseColor(ph)

switch upper(string(ph))
    case "TAKEOFF"
        c = [0.00 0.45 0.74];
    case "TOP_HOVER"
        c = [0.85 0.33 0.10];
    case "CRUISE"
        c = [0.93 0.69 0.13];
    case "TURN"
        c = [0.49 0.18 0.56];
    case "APPROACH"
        c = [0.47 0.67 0.19];
    case "DESCENT"
        c = [0.30 0.75 0.93];
    case "BOTTOM_HOVER"
        c = [0.64 0.08 0.18];
    otherwise
        c = [0.20 0.20 0.20];
end

end

function tFirst = getFirstPhaseTime(truth, phaseName)

idx = find(string(truth.phase(:)) == string(phaseName), 1, 'first');

if isempty(idx)
    tFirst = nan;
else
    tFirst = truth.t(idx);
end

end

function drawCuboidNoLegend(ax, center, sizeXYZ, faceColor, faceAlpha)

cx = center(1);
cy = center(2);
cz = center(3);

dx = sizeXYZ(1)/2;
dy = sizeXYZ(2)/2;
dz = sizeXYZ(3)/2;

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

patch(ax, ...
    'Vertices',V, ...
    'Faces',F, ...
    'FaceColor',faceColor, ...
    'FaceAlpha',faceAlpha, ...
    'EdgeColor',[0.1 0.1 0.1], ...
    'LineWidth',0.8, ...
    'HandleVisibility','off');

end