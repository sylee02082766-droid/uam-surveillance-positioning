function runMonteCarlo_A1_A2()
% RUNMONTECARLO_A1_A2
% ------------------------------------------------------------
% Robustness evaluation for Baseline / A1 / A2.
%
% What this script tests:
%   1) TOA-noise Monte Carlo via deterministic per-step measurement seeds.
%   2) Terminal outage/geometry perturbation via terminal link dropout and
%      optional forced receiver unavailability.
%   3) Truth/reference separation: actual truth is perturbed, but A1/A2 use
%      a nominal planned priorRef generated from the unperturbed cfgPrior.
%
% Outputs:
%   A1A2_MC_trials.csv
%   A1A2_MC_summary.csv
%   fig_MC_terminalP95_box.png        (if boxchart is available)
% ------------------------------------------------------------

clearvars -except ans;
close all;
clc;
addpath(pwd);

%% User options
Nmc = 50;                 % 30 is acceptable for a quick run; 50 is stronger for the paper.
base_seed = 20260426;
failure_threshold_m = 100;

% Perturbation levels requested in the discussion.
opts.landing_sigma_m = 20;             % landing point offset: sigma = 20 m
opts.descent_scale_min = 0.9;          % descent duration scale: 0.9-1.1
opts.descent_scale_max = 1.1;
opts.timing_offset_min_s = -10;        % approach/descent timing offset: +/-10 s
opts.timing_offset_max_s =  10;

% Extra Monte Carlo measurement/outage perturbations.
% These are not profile mismatch. They test receiver-count/outage robustness.
opts.toa_noise_scale_min = 0.8;        % optional TOA noise severity scale
opts.toa_noise_scale_max = 1.2;
opts.terminal_dropout_min = 0.00;      % random missed-detection probability in terminal phases
opts.terminal_dropout_max = 0.10;
opts.force_drop_probability = 0.25;    % probability of forcing one terminal receiver unavailable

%% Nominal configuration and fixed obstacle data
cfgNom = config_uam();
cfgNom = applyMonteCarloDefaults(cfgNom);
cfgNom.building_filename = 'filtered_buiding_magok_b_box.txt';
cfgNom.mountain_filename = 'mountain_pos_height.txt';

[buildings_for_physics, ~, mountains_for_physics] = loadObstacleData(cfgNom);

% This is the nominal flight-plan reference used by A1/A2.
cfgPrior = cfgNom;
priorRef = generateTruth3D(cfgPrior);
tDescNominal = firstPhaseTime(priorRef, "DESCENT");

%% Storage
trialCol = [];
methodCol = strings(0,1);
rmseCol = [];
p95Col = [];
termP95Col = [];
maxErrCol = [];
minNRxTermCol = [];
failureCol = [];

landingDxCol = [];
landingDyCol = [];
descScaleCol = [];
timingOffsetCol = [];
toaScaleCol = [];
dropProbCol = [];
forceDropCol = strings(0,1);

methods = ["Baseline", "A1", "A2"];

fprintf('\nRunning %d Monte Carlo trials...\n', Nmc);
fprintf('Truth is perturbed; A1/A2 priorRef remains nominal.\n');

for imc = 1:Nmc
    rng(base_seed + imc, 'twister');
    pert = drawPerturbation(opts, cfgNom.num_mlat);

    cfgTruth = applyTruthPerturbation(cfgNom, cfgPrior, pert, tDescNominal);
    truth = generateTruth3D(cfgTruth);

    % Same measurement seed and outage settings for all three cases.
    meas_seed = base_seed*10 + imc;
    [cfgBase, cfgA1, cfgA2] = makeCaseConfigs(cfgNom, meas_seed, pert);

    % Apply TOA-noise severity scale equally to all cases in the same trial.
    cfgBase.base_toa_noise_ns_ref = cfgBase.base_toa_noise_ns_ref * pert.toa_noise_scale;
    cfgA1.base_toa_noise_ns_ref   = cfgA1.base_toa_noise_ns_ref   * pert.toa_noise_scale;
    cfgA2.base_toa_noise_ns_ref   = cfgA2.base_toa_noise_ns_ref   * pert.toa_noise_scale;

    res = cell(3,1);
    res{1} = runSingleEKF(truth, cfgBase, buildings_for_physics, mountains_for_physics, priorRef);
    res{2} = runSingleEKF(truth, cfgA1,   buildings_for_physics, mountains_for_physics, priorRef);
    res{3} = runSingleEKF(truth, cfgA2,   buildings_for_physics, mountains_for_physics, priorRef);

    for im = 1:3
        m = computeMetrics(res{im}, truth, failure_threshold_m);

        trialCol(end+1,1) = imc; %#ok<AGROW>
        methodCol(end+1,1) = methods(im); %#ok<AGROW>
        rmseCol(end+1,1) = m.rmse; %#ok<AGROW>
        p95Col(end+1,1) = m.p95; %#ok<AGROW>
        termP95Col(end+1,1) = m.termP95; %#ok<AGROW>
        maxErrCol(end+1,1) = m.maxErr; %#ok<AGROW>
        minNRxTermCol(end+1,1) = m.minNRxTerm; %#ok<AGROW>
        failureCol(end+1,1) = m.failure; %#ok<AGROW>

        landingDxCol(end+1,1) = pert.landing_offset_xy(1); %#ok<AGROW>
        landingDyCol(end+1,1) = pert.landing_offset_xy(2); %#ok<AGROW>
        descScaleCol(end+1,1) = pert.descent_time_scale; %#ok<AGROW>
        timingOffsetCol(end+1,1) = pert.timing_offset_s; %#ok<AGROW>
        toaScaleCol(end+1,1) = pert.toa_noise_scale; %#ok<AGROW>
        dropProbCol(end+1,1) = pert.terminal_dropout_prob; %#ok<AGROW>
        forceDropCol(end+1,1) = idsToString(pert.force_drop_site_ids); %#ok<AGROW>
    end

    fprintf('  trial %3d/%3d | landing=(%+.1f,%+.1f)m | descScale=%.2f | timeOffset=%+.1fs | drop=%.2f\n', ...
        imc, Nmc, pert.landing_offset_xy(1), pert.landing_offset_xy(2), ...
        pert.descent_time_scale, pert.timing_offset_s, pert.terminal_dropout_prob);
end

Ttrial = table(trialCol, methodCol, rmseCol, p95Col, termP95Col, maxErrCol, minNRxTermCol, failureCol, ...
    landingDxCol, landingDyCol, descScaleCol, timingOffsetCol, toaScaleCol, dropProbCol, forceDropCol, ...
    'VariableNames', {'Trial','Method','RMSE_m','P95_m','TerminalP95_m','MaxError_m','MinNRxTerminal','Failure', ...
    'LandingDx_m','LandingDy_m','DescentTimeScale','TimingOffset_s','TOANoiseScale','TerminalDropoutProb','ForceDropSiteIDs'});

writetable(Ttrial, 'A1A2_MC_trials.csv');
fprintf('\nSaved: A1A2_MC_trials.csv\n');

Tsummary = makeSummaryTable(Ttrial, methods);
writetable(Tsummary, 'A1A2_MC_summary.csv');
fprintf('Saved: A1A2_MC_summary.csv\n\n');
disp(Tsummary);

% Optional plot for the paper or internal check.
try
    fig = figure('Color','w','Position',[200 200 760 420]);
    boxchart(categorical(Ttrial.Method), Ttrial.TerminalP95_m);
    grid on; box on;
    ylabel('Terminal P95 error [m]');
    title(sprintf('Monte Carlo terminal robustness, N = %d', Nmc));
    yline(failure_threshold_m, '--', sprintf('failure threshold = %g m', failure_threshold_m));
    exportgraphics(fig, 'fig_MC_terminalP95_box.png', 'Resolution', 300);
    fprintf('Saved: fig_MC_terminalP95_box.png\n');
catch ME
    fprintf('boxchart/export failed or unavailable: %s\n', ME.message);
end

end

%% ========================================================================
% Case configuration
% ========================================================================
function [cfgBase, cfgA1, cfgA2] = makeCaseConfigs(cfgNom, measurement_seed, pert)

cfgBase = cfgNom;
cfgBase.case_name = 'Baseline';
cfgBase.enable_phase_aware_q = false;
cfgBase.enable_phase_prior = false;
cfgBase.enable_terminal_degraded_update = false;
cfgBase.enable_terminal_motion_prior = false;
cfgBase.prior_variant = 'NONE';

cfgA1 = cfgNom;
cfgA1.case_name = 'A1';
cfgA1.enable_phase_aware_q = true;
cfgA1.enable_phase_prior = true;
cfgA1.enable_terminal_degraded_update = false;
cfgA1.enable_terminal_motion_prior = false;
cfgA1.prior_variant = 'A1';

cfgA2 = cfgNom;
cfgA2.case_name = 'A2';
cfgA2.enable_phase_aware_q = true;
cfgA2.enable_phase_prior = true;
cfgA2.enable_terminal_degraded_update = true;
cfgA2.enable_terminal_motion_prior = false;
cfgA2.prior_variant = 'A2';

C = {cfgBase, cfgA1, cfgA2};
for i = 1:3
    C{i}.measurement_seed = measurement_seed;
    C{i}.enable_terminal_link_dropout = true;
    C{i}.terminal_link_dropout_prob = pert.terminal_dropout_prob;
    C{i}.terminal_force_drop_site_ids = pert.force_drop_site_ids;
end
cfgBase = C{1}; cfgA1 = C{2}; cfgA2 = C{3};

end

%% ========================================================================
% Perturbation model
% ========================================================================
function pert = drawPerturbation(opts, num_mlat)
pert.landing_offset_xy = opts.landing_sigma_m * randn(1,2);
pert.descent_time_scale = opts.descent_scale_min + ...
    (opts.descent_scale_max - opts.descent_scale_min) * rand;
pert.timing_offset_s = opts.timing_offset_min_s + ...
    (opts.timing_offset_max_s - opts.timing_offset_min_s) * rand;
pert.toa_noise_scale = opts.toa_noise_scale_min + ...
    (opts.toa_noise_scale_max - opts.toa_noise_scale_min) * rand;
pert.terminal_dropout_prob = opts.terminal_dropout_min + ...
    (opts.terminal_dropout_max - opts.terminal_dropout_min) * rand;

if rand < opts.force_drop_probability
    pert.force_drop_site_ids = randi(num_mlat,1,1);
else
    pert.force_drop_site_ids = [];
end
end

function cfgTruth = applyTruthPerturbation(cfgNom, cfgPrior, pert, tDescNominal)
% Actual truth is perturbed, while cfgPrior/priorRef stays nominal.

cfgTruth = cfgNom;

% 1) Landing point offset.
cfgTruth.uam_waypoints(end,:) = cfgTruth.uam_waypoints(end,:) + pert.landing_offset_xy;

% 2) Descent duration mismatch.
cfgTruth.descent_time_s = cfgPrior.descent_time_s * pert.descent_time_scale;

% 3) Approach/descent timing offset. We tune the truth max-cruise speed so
% that the first DESCENT time is approximately shifted by the requested
% offset relative to the nominal planned reference.
tTarget = tDescNominal + pert.timing_offset_s;
cfgTruth = tuneMaxCruiseSpeedForDescentTime(cfgTruth, tTarget);
end

function cfgOut = tuneMaxCruiseSpeedForDescentTime(cfgIn, tTarget)
baseSpeed = cfgIn.max_cruise_speed;
lo = 0.70 * baseSpeed;  % slower -> later descent
hi = 1.30 * baseSpeed;  % faster -> earlier descent

bestCfg = cfgIn;
bestErr = inf;

for it = 1:18
    mid = 0.5*(lo + hi);
    cfgTest = cfgIn;
    cfgTest.max_cruise_speed = mid;
    tr = generateTruth3D(cfgTest);
    tDesc = firstPhaseTime(tr, "DESCENT");
    e = abs(tDesc - tTarget);
    if e < bestErr
        bestErr = e;
        bestCfg = cfgTest;
    end

    if tDesc > tTarget
        % Descent is too late -> speed up.
        lo = mid;
    else
        % Descent is too early -> slow down.
        hi = mid;
    end
end

cfgOut = bestCfg;
end

%% ========================================================================
% Metrics and summary
% ========================================================================
function m = computeMetrics(res, truth, failure_threshold_m)
e = res.err(:);
termMask = ismember(string(truth.phase), ["APPROACH","DESCENT","BOTTOM_HOVER"]);

m.rmse = sqrt(mean(e.^2, 'omitnan'));
m.p95 = prctile(e, 95);
m.termP95 = prctile(e(termMask), 95);
m.maxErr = max(e);
m.minNRxTerm = min(res.nRx(termMask));
m.failure = m.termP95 > failure_threshold_m;
end

function Tsummary = makeSummaryTable(Ttrial, methods)
Method = strings(numel(methods),1);
RMSE_median = zeros(numel(methods),1);
RMSE_p95 = zeros(numel(methods),1);
P95_median = zeros(numel(methods),1);
TerminalP95_median = zeros(numel(methods),1);
TerminalP95_p95 = zeros(numel(methods),1);
MaxError_median = zeros(numel(methods),1);
FailureRate = zeros(numel(methods),1);

for i = 1:numel(methods)
    idx = string(Ttrial.Method) == methods(i);
    Method(i) = methods(i);
    RMSE_median(i) = median(Ttrial.RMSE_m(idx), 'omitnan');
    RMSE_p95(i) = prctile(Ttrial.RMSE_m(idx), 95);
    P95_median(i) = median(Ttrial.P95_m(idx), 'omitnan');
    TerminalP95_median(i) = median(Ttrial.TerminalP95_m(idx), 'omitnan');
    TerminalP95_p95(i) = prctile(Ttrial.TerminalP95_m(idx), 95);
    MaxError_median(i) = median(Ttrial.MaxError_m(idx), 'omitnan');
    FailureRate(i) = mean(Ttrial.Failure(idx));
end

Tsummary = table(Method, RMSE_median, RMSE_p95, P95_median, ...
    TerminalP95_median, TerminalP95_p95, MaxError_median, FailureRate);
end

%% ========================================================================
% Defaults and utilities
% ========================================================================
function cfg = applyMonteCarloDefaults(cfg)
% Make sure newly introduced fields exist even if config_uam.m has not been
% updated yet.
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

function t = firstPhaseTime(truth, phaseName)
idx = find(string(truth.phase) == string(phaseName), 1, 'first');
if isempty(idx)
    t = truth.t(end);
else
    t = truth.t(idx);
end
end

function s = idsToString(ids)
if isempty(ids)
    s = "";
else
    s = strjoin(string(ids(:).'), ',');
end
end
