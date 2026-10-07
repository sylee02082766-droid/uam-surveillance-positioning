function result = runSingleEKF(truth, cfg, buildings_for_physics, mountains_for_physics, priorRef)
% RUNSINGLEEKF  CV-EKF with optional A1/A2 terminal enhancements.
%
% IMPORTANT FOR FAIR COMPARISON
%   truth    : actual trajectory used only for measurement generation and error evaluation.
%   priorRef : nominal/planned trajectory used by phase-aware Q and terminal priors.
%
% If priorRef is omitted, the function falls back to truth for backward
% compatibility, but for Monte Carlo/profile-mismatch studies priorRef should
% be generated from the nominal flight plan and passed explicitly.

if nargin < 5 || isempty(priorRef)
    priorRef = truth;
end

N = truth.N;
dt = cfg.dt;

% Initial state. Position is assumed initialized from departure fix. If you
% want initialization uncertainty, add it in cfg.initial_pos_sigma_m and
% cfg.initial_vel_sigma_m.
x0 = [truth.pos(1,:)'; 0; 0; 0];
if isfield(cfg,'initial_pos_sigma_m') && cfg.initial_pos_sigma_m > 0
    x0(1:3) = x0(1:3) + cfg.initial_pos_sigma_m * randn(3,1);
end
if isfield(cfg,'initial_vel_sigma_mps') && cfg.initial_vel_sigma_mps > 0
    x0(4:6) = x0(4:6) + cfg.initial_vel_sigma_mps * randn(3,1);
end
x = x0;

P = diag([10 10 10 30 30 30].^2);

F = [eye(3), dt*eye(3);
     zeros(3), eye(3)];
Q_default = diag([0.3 0.3 0.2 1.5 1.5 1.0].^2);

est = zeros(N,3);
err = nan(N,1);
pred_err = nan(N,1);
gdop = nan(N,1);
nRx = zeros(N,1);
phase_used = strings(N,1);
truth_phase = strings(N,1);

use_terminal_motion_prior = isfield(cfg,'enable_terminal_motion_prior') && cfg.enable_terminal_motion_prior;

for k = 1:N
    true_pos = truth.pos(k,:);
    truth_phase_k = string(truth.phase(k));

    kref = min(k, priorRef.N);
    phase_k = string(priorRef.phase(kref));       % phase known from planned reference, not truth
    phase_tau = priorRef.phaseTau(kref);

    phase_used(k) = phase_k;
    truth_phase(k) = truth_phase_k;

    %% 1) Prediction
    x_pred = F * x;
    if isfield(cfg, 'enable_phase_aware_q') && cfg.enable_phase_aware_q
        Q = getPhaseAwareQ6(cfg, phase_k);
    else
        Q = Q_default;
    end
    P_pred = F * P * F' + Q;

    % Legacy terminal mean-shaping is kept behind an explicit flag. It is
    % OFF in the fair Baseline/A1/A2 comparison so that A1 improvement comes
    % only from buildPhasePrior6 soft priors.
    if use_terminal_motion_prior && phase_k == "APPROACH"
        if isfield(cfg, 'approach_mean_blend')
            alpha = cfg.approach_mean_blend;
            vref_xy = priorRef.vel(kref,1:2)';
            x_pred(4:5) = (1-alpha) * x_pred(4:5) + alpha * vref_xy;
        end
        P_pred(4:5,4:5) = 0.90 * P_pred(4:5,4:5);
    end

    if use_terminal_motion_prior && any(phase_k == ["DESCENT","BOTTOM_HOVER"])
        x_pred(4:5) = 0.70 * x_pred(4:5);
        P_pred(4:5,4:5) = 0.85 * P_pred(4:5,4:5);
        if phase_k == "BOTTOM_HOVER"
            x_pred(6) = 0.50 * x_pred(6);
            P_pred(6,6) = 0.80 * P_pred(6,6);
        end
    end

    pred_err(k) = norm(true_pos - x_pred(1:3)');

    %% 2) Measurement update: strict 4-Rx or terminal degraded 3-Rx
    min_detect = 4;
    if isfield(cfg,'enable_terminal_degraded_update') && cfg.enable_terminal_degraded_update ...
            && any(phase_k == ["APPROACH","DESCENT","BOTTOM_HOVER"])
        min_detect = cfg.terminal_min_detect;
    end

    meas = getTDOAMeasurementFlexible(true_pos, cfg, buildings_for_physics, mountains_for_physics, ...
                                      min_detect, truth_phase_k, k);

    if meas.valid
        [z_hat, H] = h_tdoa_6d(x_pred, meas.refSite, meas.otherSites);
        R_eff = meas.R;

        if min_detect < 4 && meas.num_detected == cfg.terminal_min_detect
            R_eff = cfg.degraded_R_inflation * R_eff;
        end

        S = ensurePD(H * P_pred * H' + R_eff);
        innov = meas.z - z_hat;

        % Innovation clipping for weak-geometry degraded updates.
        if min_detect < 4 && meas.num_detected == cfg.terminal_min_detect
            nis = innov' / S * innov;
            if nis > cfg.degraded_nis_clip
                innov = innov * sqrt(cfg.degraded_nis_clip / nis);
            end
        end

        K = P_pred * H' / S;
        x = x_pred + K * innov;

        I = eye(size(P_pred));
        P = (I - K*H) * P_pred * (I - K*H)' + K*R_eff*K';
        gdop(k) = computeWGDOP(H(:,1:3), R_eff);
    else
        x = x_pred;
        P = P_pred;
    end
    P = ensurePD(P);

    %% 3) Phase-dependent soft-prior update
    if isfield(cfg, 'enable_phase_prior') && cfg.enable_phase_prior
        priorList = buildPhasePrior6(x, phase_k, phase_tau, meas.num_detected, cfg, k, priorRef);
        for ip = 1:numel(priorList)
            [x, P] = applySoftPriorUpdate(x, P, priorList{ip}.z, priorList{ip}.H, priorList{ip}.R);
        end
    end

    est(k,:) = x(1:3)';
    err(k) = norm(true_pos - est(k,:));
    nRx(k) = meas.num_detected;
end

result.name = 'EKF';
result.est = est;
result.err = err;
result.pred_err = pred_err;
result.gdop = gdop;
result.nRx = nRx;
result.phase_used = phase_used;
result.truth_phase = truth_phase;
result.modeProb = [];

end
