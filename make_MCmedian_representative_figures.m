function make_MCmedian_representative_figures()
% make_MCmedian_representative_figures
% ------------------------------------------------------------
% 목적:
%   1) A1A2_MC_trials.csv에서 A2의 TerminalP95 중앙값 근처 trial 선택
%   2) 그 trial을 다시 재현
%   3) (a) 장애물 포함 환경 그림
%   4) (b) 대표 수신기 수 감소 사례 그림
% 저장:
%   fig1a_env_obstacles_MCmedian.png
%   fig1b_error_nrx_MCmedian.png
% ------------------------------------------------------------

clearvars -except ans;
close all;
clc;
addpath(pwd);

%% 0) CSV 확인
csvName = 'A1A2_MC_trials.csv';
if ~isfile(csvName)
    error('%s 파일이 없습니다. 먼저 runMonteCarlo_A1_A2 를 실행하세요.', csvName);
end

T = readtable(csvName);

% Method 열 문자열 정리
if iscell(T.Method)
    methodStr = string(T.Method);
else
    methodStr = string(T.Method);
end

% 기존 Monte Carlo는 A2 이름을 사용했으므로 A2 행 선택
idxProp = methodStr == "A2";
Tprop = T(idxProp,:);

if isempty(Tprop)
    error('A2 방법의 Monte Carlo 결과를 찾지 못했습니다.');
end

targetMedian = median(Tprop.TerminalP95_m, 'omitnan');
[~, k] = min(abs(Tprop.TerminalP95_m - targetMedian));
rowChosen = Tprop(k,:);

trialID = rowChosen.Trial;
fprintf('\n선택된 대표 trial = %d\n', trialID);
fprintf('제안 기법 Terminal P95 = %.3f m (중앙값 %.3f m 근처)\n', ...
    rowChosen.TerminalP95_m, targetMedian);

%% 1) 기본 설정 및 장애물 로드
cfgNom = config_uam();
cfgNom = applyMonteCarloDefaults_local(cfgNom);
cfgNom.building_filename = 'filtered_buiding_magok_b_box.txt';
cfgNom.mountain_filename = 'mountain_pos_height.txt';

[buildings_for_physics, buildings_for_draw, mountains_for_physics] = loadObstacleData(cfgNom);

cfgPrior = cfgNom;
priorRef = generateTruth3D(cfgPrior);
tDescNominal = firstPhaseTime_local(priorRef, "DESCENT");

%% 2) CSV에서 perturbation 복원
pert.landing_offset_xy = [rowChosen.LandingDx_m, rowChosen.LandingDy_m];
pert.descent_time_scale = rowChosen.DescentTimeScale;
pert.timing_offset_s = rowChosen.TimingOffset_s;
pert.toa_noise_scale = rowChosen.TOANoiseScale;
pert.terminal_dropout_prob = rowChosen.TerminalDropoutProb;
pert.force_drop_site_ids = parseIdString_local(rowChosen.ForceDropSiteIDs);

%% 3) 실제 truth 생성
cfgTruth = applyTruthPerturbation_local(cfgNom, cfgPrior, pert, tDescNominal);
truth = generateTruth3D(cfgTruth);

%% 4) 기본 EKF / 제안 기법 설정
measurement_seed = 20260426 * 10 + trialID;

[cfgBase, cfgProp] = makeCaseConfigs_local(cfgNom, measurement_seed, pert);

% noise scale 반영
cfgBase.base_toa_noise_ns_ref = cfgBase.base_toa_noise_ns_ref * pert.toa_noise_scale;
cfgProp.base_toa_noise_ns_ref = cfgProp.base_toa_noise_ns_ref * pert.toa_noise_scale;

%% 5) 실행
resBase = runSingleEKF(truth, cfgBase, buildings_for_physics, mountains_for_physics, priorRef);
resProp = runSingleEKF(truth, cfgProp, buildings_for_physics, mountains_for_physics, priorRef);

%% 6) (a) 장애물 포함 환경 그림
figA = plotRouteAndProfiles_korean_obstacles(truth, cfgTruth, buildings_for_draw, mountains_for_physics);
exportgraphics(figA, 'fig1a_env_obstacles_MCmedian.png', 'Resolution', 300);
disp('Saved: fig1a_env_obstacles_MCmedian.png');

%% 7) (b) 대표 결과 그림
figB = plotRepresentativeComparison_local(truth, resBase, resProp, trialID, rowChosen.TerminalP95_m, targetMedian);
exportgraphics(figB, 'fig1b_error_nrx_MCmedian.png', 'Resolution', 300);
disp('Saved: fig1b_error_nrx_MCmedian.png');

end

%% ========================================================================
function [cfgBase, cfgProp] = makeCaseConfigs_local(cfgNom, measurement_seed, pert)

cfgBase = cfgNom;
cfgBase.case_name = '기본 EKF';
cfgBase.enable_phase_aware_q = false;
cfgBase.enable_phase_prior = false;
cfgBase.enable_terminal_degraded_update = false;
cfgBase.enable_terminal_motion_prior = false;
cfgBase.prior_variant = 'NONE';

cfgProp = cfgNom;
cfgProp.case_name = '제안 기법';
cfgProp.enable_phase_aware_q = true;
cfgProp.enable_phase_prior = true;
cfgProp.enable_terminal_degraded_update = true;
cfgProp.enable_terminal_motion_prior = false;
cfgProp.prior_variant = 'A2';

cfgBase.measurement_seed = measurement_seed;
cfgProp.measurement_seed = measurement_seed;

cfgBase.enable_terminal_link_dropout = true;
cfgProp.enable_terminal_link_dropout = true;

cfgBase.terminal_link_dropout_prob = pert.terminal_dropout_prob;
cfgProp.terminal_link_dropout_prob = pert.terminal_dropout_prob;

cfgBase.terminal_force_drop_site_ids = pert.force_drop_site_ids;
cfgProp.terminal_force_drop_site_ids = pert.force_drop_site_ids;

end

function cfg = applyMonteCarloDefaults_local(cfg)
if ~isfield(cfg,'enable_terminal_motion_prior'), cfg.enable_terminal_motion_prior = false; end
if ~isfield(cfg,'ref_profile_sigma_xy'), cfg.ref_profile_sigma_xy = 20.0; end
if ~isfield(cfg,'ref_profile_sigma_z'), cfg.ref_profile_sigma_z = 5.0; end
if ~isfield(cfg,'ref_profile_sigma_vxy'), cfg.ref_profile_sigma_vxy = 1.0; end
if ~isfield(cfg,'ref_profile_sigma_vz'), cfg.ref_profile_sigma_vz = 0.5; end
if ~isfield(cfg,'measurement_seed'), cfg.measurement_seed = []; end
if ~isfield(cfg,'enable_terminal_link_dropout'), cfg.enable_terminal_link_dropout = false; end
if ~isfield(cfg,'terminal_link_dropout_prob'), cfg.terminal_link_dropout_prob = 0.0; end
if ~isfield(cfg,'terminal_force_drop_site_ids'), cfg.terminal_force_drop_site_ids = []; end
if ~isfield(cfg,'initial_pos_sigma_m'), cfg.initial_pos_sigma_m = 0.0; end
if ~isfield(cfg,'initial_vel_sigma_mps'), cfg.initial_vel_sigma_mps = 0.0; end
end

function cfgTruth = applyTruthPerturbation_local(cfgNom, cfgPrior, pert, tDescNominal)
cfgTruth = cfgNom;

% 1) 착륙점 offset
cfgTruth.uam_waypoints(end,:) = cfgTruth.uam_waypoints(end,:) + pert.landing_offset_xy;

% 2) 하강 시간 scale
cfgTruth.descent_time_s = cfgPrior.descent_time_s * pert.descent_time_scale;

% 3) 접근/하강 타이밍 offset
tTarget = tDescNominal + pert.timing_offset_s;
cfgTruth = tuneMaxCruiseSpeedForDescentTime_local(cfgTruth, tTarget);
end

function cfgOut = tuneMaxCruiseSpeedForDescentTime_local(cfgIn, tTarget)
baseSpeed = cfgIn.max_cruise_speed;
lo = 0.70 * baseSpeed;
hi = 1.30 * baseSpeed;

bestCfg = cfgIn;
bestErr = inf;

for it = 1:18
    mid = 0.5*(lo+hi);
    cfgTest = cfgIn;
    cfgTest.max_cruise_speed = mid;

    tr = generateTruth3D(cfgTest);
    tDesc = firstPhaseTime_local(tr, "DESCENT");
    e = abs(tDesc - tTarget);

    if e < bestErr
        bestErr = e;
        bestCfg = cfgTest;
    end

    if tDesc > tTarget
        lo = mid;
    else
        hi = mid;
    end
end

cfgOut = bestCfg;
end

function t = firstPhaseTime_local(truth, phaseName)
idx = find(string(truth.phase) == string(phaseName), 1, 'first');
if isempty(idx)
    t = truth.t(end);
else
    t = truth.t(idx);
end
end

function ids = parseIdString_local(x)
if ismissing(x) || isempty(x)
    ids = [];
    return;
end

if iscell(x)
    s = string(x{1});
else
    s = string(x);
end

if strlength(strtrim(s)) == 0
    ids = [];
    return;
end

parts = split(s, ',');
ids = str2double(parts);
ids = ids(~isnan(ids));
ids = ids(:).';
end

%% ========================================================================
function fig = plotRepresentativeComparison_local(truth, resBase, resProp, trialID, termP95_trial, termP95_median)

idxAll = 1:numel(truth.t);
tAll = truth.t(idxAll);

errBase = resBase.err(idxAll);
errProp = resProp.err(idxAll);
nRx     = resBase.nRx(idxAll);

% ------------------------------------------------------------
% 착륙 단계 주변만 보여주기
% APPROACH, DESCENT, BOTTOM_HOVER를 terminal window로 사용
% ------------------------------------------------------------
phase = string(truth.phase(:));
termMask = ismember(phase, ["APPROACH","DESCENT","BOTTOM_HOVER"]);

idxTerm = find(termMask);

if isempty(idxTerm)
    % fallback
    xWin = [max(0, tAll(end)-80), tAll(end)];
else
    tStart = truth.t(idxTerm(1));
    tEnd   = truth.t(idxTerm(end));

    % 앞뒤 여유
    xWin = [max(0, tStart - 20), min(tAll(end), tEnd + 5)];
end

fig = figure('Color','w','Position',[80 80 1700 820]);

ax1 = axes('Parent', fig, ...
    'Units','normalized', ...
    'Position', [0.060 0.570 0.910 0.380]);

ax2 = axes('Parent', fig, ...
    'Units','normalized', ...
    'Position', [0.060 0.105 0.910 0.360]);

%% ------------------------------------------------------------
% (a) 추정 오차
% ------------------------------------------------------------
hold(ax1,'on');
grid(ax1,'on');
box(ax1,'on');

plot(ax1, tAll, errBase, '-', ...
    'LineWidth', 2.6, ...
    'Color', [0.00 0.45 0.74]);

plot(ax1, tAll, errProp, '-', ...
    'LineWidth', 2.6, ...
    'Color', [0.85 0.33 0.10]);

xlabel(ax1,'시간 [s]','FontSize',12);
ylabel(ax1,'추정 오차 [m]','FontSize',12);

title(ax1, sprintf(['(b-1) 추정 오차'], ...
    trialID, termP95_trial), ...
    'FontSize',14, ...
    'FontWeight','bold');

legend(ax1, {'기본 EKF','제안 기법'}, ...
    'Location','northwest', ...
    'FontSize',12);

set(ax1,'FontSize',11);
xlim(ax1, xWin);

% y축은 현재 window 안의 최대값 기준으로 설정
idxWin = tAll >= xWin(1) & tAll <= xWin(2);
maxErrWin = max([errBase(idxWin); errProp(idxWin)], [], 'omitnan');

ylim(ax1, [0, max(150, ceil(maxErrWin/100)*100)]);

addPhaseGuideLines_local(ax1, truth);

%% ------------------------------------------------------------
% (b) 검출 수신기 수
% ------------------------------------------------------------
hold(ax2,'on');
grid(ax2,'on');
box(ax2,'on');

stairs(ax2, tAll, nRx, 'k-', 'LineWidth', 2.4);

xlabel(ax2,'시간 [s]','FontSize',12);
ylabel(ax2,'검출 수신기 수','FontSize',12);
title(ax2,'(b-2) 검출 수신기 수','FontSize',14,'FontWeight','bold');

xlim(ax2, xWin);
ylim(ax2, [0.5, max(4.6, max(nRx(idxWin))+0.5)]);

yline(ax2, 4, '--', ...
    '4개 수신기 기준', ...
    'Color',[0.90 0.20 0.20], ...
    'LineWidth',1.6, ...
    'FontSize',10, ...
    'LabelHorizontalAlignment','left');

yline(ax2, 3, ':', ...
    '3개 수신기 보수적 보정 기준', ...
    'Color',[0.00 0.65 0.00], ...
    'LineWidth',1.8, ...
    'FontSize',10, ...
    'LabelHorizontalAlignment','left');

legend(ax2, {'검출 수신기 수'}, ...
    'Location','northeast', ...
    'FontSize',12);

set(ax2,'FontSize',11);

addPhaseGuideLines_local(ax2, truth);

linkaxes([ax1 ax2], 'x');

end

function addPhaseGuideLines_local(ax, truth)
phaseLocal = string(truth.phase(:));
tLocal = truth.t(:);

segments = findPhaseSegments_local2(phaseLocal);
yl = ylim(ax);
yr = yl(2)-yl(1);

for i = 1:numel(segments)
    segName = segments(i).name;
    segIdx  = segments(i).idx;

    segStartTime = tLocal(segIdx(1));
    segMidTime   = tLocal(round((segIdx(1)+segIdx(end))/2));

    c = phaseColor_local2(segName);
    txt = prettyPhaseName_local(segName);

    if i >= 2
        xline(ax, segStartTime, '--', 'Color', c, 'LineWidth', 1.1, 'HandleVisibility','off');
    end

    yText = yl(2) - 0.08*yr;
    text(ax, segMidTime, yText, txt, ...
        'Color', c, 'FontSize', 9, 'FontWeight','bold', ...
        'HorizontalAlignment','center', ...
        'VerticalAlignment','top', ...
        'BackgroundColor','w', ...
        'Margin',1.0, ...
        'HandleVisibility','off');
end
end

function segments = findPhaseSegments_local2(phase)
phase = string(phase(:));
N = numel(phase);
segments = struct('name',{},'idx',{});

if N == 0, return; end
iStart = 1; cnt = 0;
for k = 2:N
    if phase(k) ~= phase(k-1)
        cnt = cnt + 1;
        segments(cnt).name = phase(iStart);
        segments(cnt).idx = iStart:(k-1);
        iStart = k;
    end
end
cnt = cnt + 1;
segments(cnt).name = phase(iStart);
segments(cnt).idx = iStart:N;
end

function c = phaseColor_local2(ph)
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

function txt = prettyPhaseName_local(ph)
switch upper(string(ph))
    case "TAKEOFF",      txt = '이륙';
    case "TOP_HOVER",    txt = '상단 정지';
    case "CRUISE",       txt = '순항';
    case "TURN",         txt = '선회';
    case "APPROACH",     txt = '접근';
    case "DESCENT",      txt = '하강';
    case "BOTTOM_HOVER", txt = '하단 정지';
    otherwise,           txt = char(string(ph));
end
end