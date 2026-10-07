function Q = getPhaseAwareQ6(cfg, phase_k)
% GETPHASEAWAREQ6  Phase-dependent process noise for the 6D CV-EKF.

switch string(phase_k)
    case "TURN"
        stds = [0.45 0.45 0.25  3.0 3.0 1.2];
    case "APPROACH"
        stds = [0.35 0.35 0.20  4.0 4.0 0.8];
    case "DESCENT"
        stds = [0.20 0.20 0.45  1.2 1.2 4.0];
    case "BOTTOM_HOVER"
        stds = [0.10 0.10 0.10  0.40 0.40 0.40];
    case "TAKEOFF"
        stds = [0.20 0.20 0.35  0.9 0.9 2.2];
    otherwise
        stds = [0.30 0.30 0.20  1.5 1.5 1.0];
end

Q = diag(stds.^2);
end
