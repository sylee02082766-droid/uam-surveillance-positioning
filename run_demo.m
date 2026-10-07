function metrics = run_demo()
% RUN_DEMO Compare CV-EKF / A1 / A2 on clearly synthetic obstacle inputs.
% The nominal reference is separate from the deliberately perturbed truth.
projectDir = fileparts(mfilename('fullpath'));
previousDir = pwd;
restoreDir = onCleanup(@() cd(previousDir)); %#ok<NASGU>
cd(projectDir);
addpath(projectDir);
cfg = config_uam();
cfg.measurement_seed = 20261007;
cfg.enable_terminal_link_dropout = true;
cfg.terminal_link_dropout_prob = 0.05;
[buildings, ~, mountains] = loadObstacleData(cfg);
priorRef = generateTruth3D(cfg);
cfgTruth = cfg;
cfgTruth.uam_waypoints(end,:) = cfgTruth.uam_waypoints(end,:) + [15, -10];
cfgTruth.descent_time_s = 1.05 * cfg.descent_time_s;
truth = generateTruth3D(cfgTruth);
methods = ["Baseline", "A1", "A2"];
rmse = zeros(3,1);
p95 = zeros(3,1);
for i = 1:3
    c = cfg;
    c.enable_phase_aware_q = i > 1;
    c.enable_phase_prior = i > 1;
    c.enable_terminal_degraded_update = i == 3;
    c.enable_terminal_motion_prior = false;
    c.prior_variant = char(methods(i));
    if i == 1, c.prior_variant = 'NONE'; end
    result = runSingleEKF(truth, c, buildings, mountains, priorRef);
    assert(all(isfinite(result.err)), 'Non-finite position error.');
    rmse(i) = sqrt(mean(result.err.^2));
    p95(i) = prctile(result.err,95);
end
metrics = table(methods', rmse, p95, 'VariableNames', {'Method','DemoRMSE_m','DemoP95_m'});
disp('SYNTHETIC-DEMO RESULTS ONLY: these are not Olympiad/paper benchmark results.');
disp(metrics);
end
