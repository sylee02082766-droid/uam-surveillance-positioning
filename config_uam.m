function cfg = config_uam()
% CONFIG_UAM  Configuration for the UAM MLAT terminal tracking simulator.
% The three representative cases are:
%   Baseline : CV-EKF + strict 4-Rx TDOA update
%   A1       : Baseline + phase-aware Q + terminal profile priors
%   A2       : A1 + 3-Rx degraded TDOA update

%% Execution options
cfg.method = 'EKF';
cfg.plot_buildings = true;
cfg.plot_fast_buildings = true;
cfg.plot_mode_prob = false;

%% Fixed MLAT receiver deployment
% The 11 virtual sites were selected by offline corridor-coverage and
% geometry screening. z is representative installation height.
final_mlat_positions = 100000 * [
    1.8652 5.5056
    1.9042 5.4831
    1.9172 5.4831
    1.8588 5.5169
    1.9367 5.4944
    1.8782 5.5281
    1.9562 5.4831
    1.9172 5.5056
    1.9042 5.5056
    1.8588 5.4944
    1.8523 5.5056
];
mlat_altitudes = [128; 132; 126; 138; 163; 125; 124; 138; 132; 124; 177];

cfg.mlat_sites_3d = [final_mlat_positions, mlat_altitudes];
cfg.num_mlat = size(cfg.mlat_sites_3d, 1);

%% UAM waypoints: V3 - JP1 - JP2 - JP3 - V4
p_v3  = [183679, 550875];
p_jp1 = [187404, 551700];
p_jp2 = [190583, 550056];
p_jp3 = [191857, 549643];
p_v4  = [193093, 546814];
cfg.uam_waypoints = [p_v3; p_jp1; p_jp2; p_jp3; p_v4];

%% 3D truth trajectory scenario
cfg.cruise_altitude_m = 450;
cfg.dt = 1.0;
cfg.takeoff_time_s = 35;
cfg.top_hover_time_s = 3;
cfg.bottom_hover_time_s = 3;
cfg.descent_time_s = 35;
cfg.path_resample_m = 10;
cfg.path_smooth_window = 21;
cfg.max_cruise_speed = 38;
cfg.max_cruise_accel = 2.0;

% Phase labelling
cfg.enable_turn_phase = true;
cfg.turn_region_radius_m = 250;
cfg.enable_approach_phase = true;

%% RF parameters
cfg.freq_MHz = 5120;
cfg.tx_power_dBm = 30;
cfg.tx_gain_dBi = 6;
cfg.rx_gain_dBi = 12;
cfg.tx_cable_loss_dB = 0.3;
cfg.rx_cable_loss_dB = 0.3;
cfg.fade_margin_dB = 3;
cfg.rx_sensitivity_dBm = -75.4;
cfg.max_reception_range_m = 3000;

cfg.bandwidth_Hz = 2e6;
cfg.noise_figure_dB = 2;
cfg.k_boltzmann = 1.38e-23;
cfg.T_kelvin = 290;

cfg.base_toa_noise_ns_ref = 5;
cfg.base_snr_ref_dB = 20;
cfg.nlos_penalty_dB = 20;
cfg.c = 299792458;

cfg.eirp_dBm = cfg.tx_power_dBm + cfg.tx_gain_dBi - cfg.tx_cable_loss_dB;
cfg.noise_floor_dBm = 10*log10(cfg.k_boltzmann * cfg.T_kelvin * cfg.bandwidth_Hz) + 30 + cfg.noise_figure_dB;

%% A1/A2 tracking options
cfg.enable_phase_aware_q = true;
cfg.enable_phase_prior = true;
cfg.prior_variant = 'A2';    % 'A1' or 'A2'
cfg.enable_terminal_degraded_update = true;
cfg.terminal_min_detect = 3;
cfg.degraded_R_inflation = 4.0;
cfg.degraded_nis_clip = 16.0;

% Legacy prediction mean-shaping. Keep this OFF for fair Baseline/A1/A2
% comparison; A1/A2 improvements should come from buildPhasePrior6.
cfg.enable_terminal_motion_prior = false;
cfg.approach_mean_blend = 0.5;

% Reference-profile uncertainty used to keep terminal priors soft under
% planned-profile mismatch. These are added in quadrature to each prior
% covariance inside buildPhasePrior6.
cfg.ref_profile_sigma_xy = 20.0;     % [m], landing/reference lateral uncertainty
cfg.ref_profile_sigma_z = 5.0;       % [m], altitude profile uncertainty
cfg.ref_profile_sigma_vxy = 1.0;     % [m/s], horizontal speed-profile uncertainty
cfg.ref_profile_sigma_vz = 0.5;      % [m/s], vertical speed-profile uncertainty

% Monte Carlo measurement perturbation options. These are disabled by
% default and enabled in runMonteCarlo_A1_A2.m.
cfg.measurement_seed = [];
cfg.enable_terminal_link_dropout = false;
cfg.terminal_link_dropout_prob = 0.0;
cfg.terminal_force_drop_site_ids = [];

% Optional initial-state perturbation. Keep zero unless you explicitly want
% to test initialization sensitivity.
cfg.initial_pos_sigma_m = 0.0;
cfg.initial_vel_sigma_mps = 0.0;

%% A1 priors
cfg.approach_prior_sigma_xy = 2.0;
cfg.approach_prior_sigma_xy_outage = 0.6;
cfg.descent_prior_sigma_xy = 2.0;
cfg.descent_prior_sigma_xy_outage = 0.6;
cfg.descent_prior_sigma_vz = 1.8;
cfg.descent_prior_sigma_vz_outage = 0.6;
cfg.descent_altitude_prior_sigma_z = 6.0;
cfg.bottom_hover_prior_sigma_v = 1.0;
cfg.bottom_hover_prior_sigma_v_outage = 0.6;
cfg.bottom_hover_prior_sigma_pos_xy = 6.0;
cfg.bottom_hover_prior_sigma_pos_z  = 3.0;

%% A2 priors: degraded TDOA + stronger terminal corridor/profile priors
cfg.approach_prior_sigma_pos_xy_outage_A2 = 8.0;
cfg.approach_prior_sigma_vxy_A2 = 2.0;
cfg.approach_prior_sigma_vxy_outage_A2 = 0.8;

cfg.descent_prior_sigma_pos_xy_A2 = 12.0;
cfg.descent_prior_sigma_pos_xy_outage_A2 = 6.0;
cfg.descent_prior_sigma_vxy_A2 = 1.5;
cfg.descent_prior_sigma_vxy_outage_A2 = 0.5;
cfg.descent_profile_sigma_z_A2 = 8.0;
cfg.descent_profile_sigma_z_outage_A2 = 4.0;
cfg.descent_profile_sigma_vz_A2 = 1.5;
cfg.descent_profile_sigma_vz_outage_A2 = 0.5;

cfg.bottom_hover_prior_sigma_v_A2 = 1.0;
cfg.bottom_hover_prior_sigma_v_outage_A2 = 0.3;
cfg.bottom_hover_prior_sigma_pos_xy_A2 = 6.0;
cfg.bottom_hover_prior_sigma_pos_xy_outage_A2 = 3.0;
cfg.bottom_hover_prior_sigma_pos_z_A2 = 3.0;
cfg.bottom_hover_prior_sigma_pos_z_outage_A2 = 1.5;

%% Obstacle data file paths
cfg.building_filename = 'filtered_buiding_magok_b_box.txt';
cfg.mountain_filename = 'mountain_pos_height.txt';

%% Visualization
cfg.building_roi_margin = 150;
cfg.min_building_height_draw = 20;

end
