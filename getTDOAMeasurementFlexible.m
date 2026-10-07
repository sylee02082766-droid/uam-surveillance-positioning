function meas = getTDOAMeasurementFlexible(current_true_pos, cfg, buildings_for_physics, mountains_for_physics, min_detect, phase_k, k_step)
% GETTDOAMEASUREMENTFLEXIBLE  MLAT TDOA measurement with configurable minimum receiver count.
%
% Optional Monte Carlo features:
%   cfg.measurement_seed              : deterministic per-step random stream
%   cfg.enable_terminal_link_dropout  : random terminal receiver/link missed detection
%   cfg.terminal_link_dropout_prob    : missed-detection probability in terminal phases
%   cfg.terminal_force_drop_site_ids  : receiver indices forcibly dropped in terminal phases

if nargin < 5 || isempty(min_detect)
    min_detect = 4;
end
if nargin < 6 || isempty(phase_k)
    phase_k = "";
end
if nargin < 7 || isempty(k_step)
    k_step = 1;
end

num_mlat = cfg.num_mlat;
mlat_sites_3d = cfg.mlat_sites_3d;
c = cfg.c;

% Deterministic random stream per time step. This keeps TOA noise/dropout
% comparable across Baseline/A1/A2 even when one method skips an update.
use_stream = isfield(cfg,'measurement_seed') && ~isempty(cfg.measurement_seed) && ~isnan(cfg.measurement_seed);
if use_stream
    seed_k = double(cfg.measurement_seed) + 1000003 * double(k_step);
    seed_k = mod(round(seed_k), 2^32-1);
    if seed_k == 0, seed_k = 1; end
    rs = RandStream('mt19937ar','Seed',seed_k);
else
    rs = [];
end

distances_m = vecnorm(mlat_sites_3d - current_true_pos, 2, 2);
path_loss_dB = 20*log10(max(distances_m,1)) + 20*log10(cfg.freq_MHz*1e6) - 147.55;

nlos_losses = zeros(num_mlat,1);
for k = 1:num_mlat
    los_build = check_los_cylinder(current_true_pos, mlat_sites_3d(k,:), buildings_for_physics);
    los_mtn   = check_los_cylinder(current_true_pos, mlat_sites_3d(k,:), mountains_for_physics);
    if ~(los_build && los_mtn)
        nlos_losses(k) = cfg.nlos_penalty_dB;
    end
end

total_loss_dB = path_loss_dB + nlos_losses;
received_power_dBm = cfg.eirp_dBm - total_loss_dB + cfg.rx_gain_dBi ...
    - cfg.rx_cable_loss_dB - cfg.fade_margin_dB;

is_detected = (received_power_dBm >= cfg.rx_sensitivity_dBm) & ...
              (distances_m <= cfg.max_reception_range_m);

% Terminal outage / geometry perturbation: simulate intermittent missed
% detections or local receiver unavailability only in terminal phases.
terminal_phase = any(string(phase_k) == ["APPROACH","DESCENT","BOTTOM_HOVER"]);
if terminal_phase && isfield(cfg,'enable_terminal_link_dropout') && cfg.enable_terminal_link_dropout
    p_drop = getFieldDefault(cfg,'terminal_link_dropout_prob',0);
    if p_drop > 0
        if use_stream
            r = rand(rs, num_mlat, 1);
        else
            r = rand(num_mlat,1);
        end
        is_detected = is_detected & (r >= p_drop);
    end

    if isfield(cfg,'terminal_force_drop_site_ids') && ~isempty(cfg.terminal_force_drop_site_ids)
        ids = cfg.terminal_force_drop_site_ids(:);
        ids = ids(ids >= 1 & ids <= num_mlat);
        is_detected(ids) = false;
    end
end

detected_indices = find(is_detected);
num_detected = length(detected_indices);

meas.valid = false;
meas.num_detected = num_detected;
meas.z = [];
meas.refSite = [];
meas.otherSites = [];
meas.R = [];
meas.snr_dB = [];
meas.detected_indices = detected_indices;

if num_detected < min_detect
    return;
end

ref_idx_local = selectReferenceReceiver(received_power_dBm(detected_indices));
reorder = [ref_idx_local, setdiff(1:num_detected, ref_idx_local, 'stable')];

detected_sites = mlat_sites_3d(detected_indices, :);
detected_sites = detected_sites(reorder, :);
detected_indices = detected_indices(reorder);

snr_dB = received_power_dBm(detected_indices) - cfg.noise_floor_dBm;
snr_linear = max(10.^(snr_dB/10), 1e-6);
base_snr_ref_linear = 10^(cfg.base_snr_ref_dB/10);

dynamic_toa_noise_ns = cfg.base_toa_noise_ns_ref * sqrt(base_snr_ref_linear ./ snr_linear);
dynamic_toa_noise_s = dynamic_toa_noise_ns * 1e-9;

true_toa = distances_m(detected_indices) / c;
if use_stream
    noise = randn(rs, num_detected, 1);
else
    noise = randn(num_detected,1);
end
noisy_toa = true_toa + noise .* dynamic_toa_noise_s;

ref_site = detected_sites(1,:);
other_sites = detected_sites(2:end,:);
z_meas = (noisy_toa(2:end) - noisy_toa(1)) * c;

sigma_range = dynamic_toa_noise_s * c;
sigma_ref = sigma_range(1);
sigma_other = sigma_range(2:end);

m = num_detected - 1;
R = zeros(m,m);
for i = 1:m
    for j = 1:m
        if i == j
            R(i,j) = sigma_other(i)^2 + sigma_ref^2;
        else
            R(i,j) = sigma_ref^2;
        end
    end
end
R = ensurePD(R);

meas.valid = true;
meas.num_detected = num_detected;
meas.z = z_meas;
meas.refSite = ref_site;
meas.otherSites = other_sites;
meas.R = R;
meas.snr_dB = snr_dB;
meas.detected_indices = detected_indices;

end

function v = getFieldDefault(s, name, defaultValue)
if isfield(s, name)
    v = s.(name);
else
    v = defaultValue;
end
end
