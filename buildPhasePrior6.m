function priorList = buildPhasePrior6(x, phase_k, phase_tau, num_detected, cfg, k, priorRef)
% BUILDPHASEPRIOR6  Phase-dependent pseudo measurements for A1/A2.
%
% priorRef must be a nominal/planned reference trajectory, not the actual
% truth trajectory. This prevents oracle use of truth.pos/truth.vel in the
% terminal prior.

if nargin < 7 || isempty(priorRef)
    error('buildPhasePrior6 requires priorRef. Pass nominal/planned reference trajectory, not truth.');
end

variant = "A2";
if isfield(cfg, 'prior_variant')
    variant = upper(string(cfg.prior_variant));
end

% Backward-compatible aliases used in old scripts.
if variant == "B7"
    variant = "A1";
elseif variant == "B8"
    variant = "A2";
end

if variant == "A1"
    priorList = buildA1(phase_k, phase_tau, num_detected, cfg, k, priorRef);
elseif variant == "A2"
    priorList = buildA2(phase_k, phase_tau, num_detected, cfg, k, priorRef);
else
    priorList = {};
end
end

%% ========================================================================
% A1: terminal profile prior only
% ========================================================================
function priorList = buildA1(phase_k, phase_tau, num_detected, cfg, k, priorRef)
priorList = {};
kref = min(k, priorRef.N);
ref_pos = priorRef.pos(kref,:)';
ref_vel = priorRef.vel(kref,:)';

switch string(phase_k)
    case "APPROACH"
        vref_xy = ref_vel(1:2);
        sig_vxy = cfg.approach_prior_sigma_xy;
        if num_detected < 4, sig_vxy = cfg.approach_prior_sigma_xy_outage; end
        sig_vxy = inflateSigma(sig_vxy, getFieldDefault(cfg,'ref_profile_sigma_vxy',0));

        p.z = vref_xy;
        p.H = [0 0 0 1 0 0; 0 0 0 0 1 0];
        p.R = diag([sig_vxy, sig_vxy].^2);
        priorList{end+1} = p;

    case "DESCENT"
        if isnan(phase_tau), phase_tau = 0; end %#ok<NASGU>

        sig_vxy = cfg.descent_prior_sigma_xy;
        if num_detected < 4, sig_vxy = cfg.descent_prior_sigma_xy_outage; end
        sig_vxy = inflateSigma(sig_vxy, getFieldDefault(cfg,'ref_profile_sigma_vxy',0));
        p1.z = ref_vel(1:2);
        p1.H = [0 0 0 1 0 0; 0 0 0 0 1 0];
        p1.R = diag([sig_vxy, sig_vxy].^2);
        priorList{end+1} = p1;

        vz_ref = ref_vel(3);
        if vz_ref > -0.5, vz_ref = -0.5; end
        sig_vz = cfg.descent_prior_sigma_vz;
        if num_detected < 4, sig_vz = cfg.descent_prior_sigma_vz_outage; end
        sig_vz = inflateSigma(sig_vz, getFieldDefault(cfg,'ref_profile_sigma_vz',0));
        p2.z = vz_ref;
        p2.H = [0 0 0 0 0 1];
        p2.R = sig_vz^2;
        priorList{end+1} = p2;

        if num_detected < 4
            sig_z = inflateSigma(cfg.descent_altitude_prior_sigma_z, getFieldDefault(cfg,'ref_profile_sigma_z',0));
            p3.z = ref_pos(3);
            p3.H = [0 0 1 0 0 0];
            p3.R = sig_z^2;
            priorList{end+1} = p3;
        end

    case "BOTTOM_HOVER"
        sig_v = cfg.bottom_hover_prior_sigma_v;
        if num_detected < 4, sig_v = cfg.bottom_hover_prior_sigma_v_outage; end
        sig_vxy = inflateSigma(sig_v, getFieldDefault(cfg,'ref_profile_sigma_vxy',0));
        sig_vz  = inflateSigma(sig_v, getFieldDefault(cfg,'ref_profile_sigma_vz',0));
        p1.z = [0; 0; 0];
        p1.H = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
        p1.R = diag([sig_vxy, sig_vxy, sig_vz].^2);
        priorList{end+1} = p1;

        if num_detected < 4
            landing_xyz = [priorRef.pos(min(kref,priorRef.N),1); priorRef.pos(min(kref,priorRef.N),2); 0];
            sig_xy = inflateSigma(cfg.bottom_hover_prior_sigma_pos_xy, getFieldDefault(cfg,'ref_profile_sigma_xy',0));
            sig_z  = inflateSigma(cfg.bottom_hover_prior_sigma_pos_z, getFieldDefault(cfg,'ref_profile_sigma_z',0));
            p2.z = landing_xyz;
            p2.H = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0];
            p2.R = diag([sig_xy, sig_xy, sig_z].^2);
            priorList{end+1} = p2;
        end
end
end

%% ========================================================================
% A2: A1-type prior + degraded 3-Rx radio correction support
% ========================================================================
function priorList = buildA2(phase_k, phase_tau, num_detected, cfg, k, priorRef)
priorList = {};
kref = min(k, priorRef.N);
ref_pos = priorRef.pos(kref,:)';
ref_vel = priorRef.vel(kref,:)';

switch string(phase_k)
    case "APPROACH"
        if num_detected < 4
            sig_pos_xy = inflateSigma(cfg.approach_prior_sigma_pos_xy_outage_A2, getFieldDefault(cfg,'ref_profile_sigma_xy',0));
            p0.z = ref_pos(1:2);
            p0.H = [1 0 0 0 0 0; 0 1 0 0 0 0];
            p0.R = diag([sig_pos_xy, sig_pos_xy].^2);
            priorList{end+1} = p0;
        end

        sig_vxy = cfg.approach_prior_sigma_vxy_A2;
        if num_detected < 4, sig_vxy = cfg.approach_prior_sigma_vxy_outage_A2; end
        sig_vxy = inflateSigma(sig_vxy, getFieldDefault(cfg,'ref_profile_sigma_vxy',0));
        p1.z = ref_vel(1:2);
        p1.H = [0 0 0 1 0 0; 0 0 0 0 1 0];
        p1.R = diag([sig_vxy, sig_vxy].^2);
        priorList{end+1} = p1;

    case "DESCENT"
        if isnan(phase_tau), phase_tau = 0; end %#ok<NASGU>

        sig_pos_xy = cfg.descent_prior_sigma_pos_xy_A2;
        if num_detected < 4, sig_pos_xy = cfg.descent_prior_sigma_pos_xy_outage_A2; end
        sig_pos_xy = inflateSigma(sig_pos_xy, getFieldDefault(cfg,'ref_profile_sigma_xy',0));
        p0.z = ref_pos(1:2);
        p0.H = [1 0 0 0 0 0; 0 1 0 0 0 0];
        p0.R = diag([sig_pos_xy, sig_pos_xy].^2);
        priorList{end+1} = p0;

        sig_vxy = cfg.descent_prior_sigma_vxy_A2;
        if num_detected < 4, sig_vxy = cfg.descent_prior_sigma_vxy_outage_A2; end
        sig_vxy = inflateSigma(sig_vxy, getFieldDefault(cfg,'ref_profile_sigma_vxy',0));
        p1.z = ref_vel(1:2);
        p1.H = [0 0 0 1 0 0; 0 0 0 0 1 0];
        p1.R = diag([sig_vxy, sig_vxy].^2);
        priorList{end+1} = p1;

        sig_z = cfg.descent_profile_sigma_z_A2;
        sig_vz = cfg.descent_profile_sigma_vz_A2;
        if num_detected < 4
            sig_z = cfg.descent_profile_sigma_z_outage_A2;
            sig_vz = cfg.descent_profile_sigma_vz_outage_A2;
        end
        sig_z  = inflateSigma(sig_z, getFieldDefault(cfg,'ref_profile_sigma_z',0));
        sig_vz = inflateSigma(sig_vz, getFieldDefault(cfg,'ref_profile_sigma_vz',0));

        vz_ref = ref_vel(3);
        if vz_ref > -0.5, vz_ref = -0.5; end
        p2.z = [ref_pos(3); vz_ref];
        p2.H = [0 0 1 0 0 0; 0 0 0 0 0 1];
        p2.R = diag([sig_z, sig_vz].^2);
        priorList{end+1} = p2;

    case "BOTTOM_HOVER"
        sig_v = cfg.bottom_hover_prior_sigma_v_A2;
        sig_xy = cfg.bottom_hover_prior_sigma_pos_xy_A2;
        sig_z = cfg.bottom_hover_prior_sigma_pos_z_A2;
        if num_detected < 4
            sig_v = cfg.bottom_hover_prior_sigma_v_outage_A2;
            sig_xy = cfg.bottom_hover_prior_sigma_pos_xy_outage_A2;
            sig_z = cfg.bottom_hover_prior_sigma_pos_z_outage_A2;
        end
        sig_vxy = inflateSigma(sig_v, getFieldDefault(cfg,'ref_profile_sigma_vxy',0));
        sig_vz  = inflateSigma(sig_v, getFieldDefault(cfg,'ref_profile_sigma_vz',0));
        sig_xy  = inflateSigma(sig_xy, getFieldDefault(cfg,'ref_profile_sigma_xy',0));
        sig_z   = inflateSigma(sig_z, getFieldDefault(cfg,'ref_profile_sigma_z',0));

        p1.z = [0; 0; 0];
        p1.H = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
        p1.R = diag([sig_vxy, sig_vxy, sig_vz].^2);
        priorList{end+1} = p1;

        landing_xyz = [ref_pos(1); ref_pos(2); 0];
        p2.z = landing_xyz;
        p2.H = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0];
        p2.R = diag([sig_xy, sig_xy, sig_z].^2);
        priorList{end+1} = p2;
end
end

%% ========================================================================
% Small utilities
% ========================================================================
function y = inflateSigma(sigma_base, sigma_ref)
y = sqrt(sigma_base.^2 + sigma_ref.^2);
end

function v = getFieldDefault(s, name, defaultValue)
if isfield(s, name)
    v = s.(name);
else
    v = defaultValue;
end
end
