# Selection of the final ICROS implementation

The selected source is the final MLAT receiver-reduction implementation, identified by agreement with the accepted ICROS 2026 paper *Robust UAM Tracking Using Prior Flight Information under Reduced MLAT Receiver Availability*. Folder names and timestamps alone were not used to decide.

## Evidence

- The final paper compares a **3D CV-EKF baseline** with soft nominal flight-plan priors and conservative **three-receiver TDOA updates**. `config_uam`, `runSingleEKF`, `buildPhasePrior6` and `getTDOAMeasurementFlexible` implement that model; this is not the earlier IMM or 2D EKF/PF study.
- `runMonteCarlo_A1_A2` runs **50 trials**, passing a separate nominal `priorRef` to each filter while perturbing the actual trajectory, measurement noise and terminal receiver availability.
- The original local Monte Carlo result files have exactly **150 rows (50 trials × Baseline/A1/A2)**. Their baseline/proposed summary matches the final paper: RMSE medians **220.35/22.44 m**, proposed landing P95 median **95.36 m**, and proposed landing P95 tail **210.65 m**. The paper's baseline landing P95 of approximately **1094.50 m** corresponds to **1094.5342 m** in the local summary (minor presentation rounding).
- `make_MCmedian_representative_figures` selects the A2 trial nearest the median landing P95. Independently applying its rule to the original local CSV selects **trial 19**, exactly the case described for Figure 1 in the final paper.
- The source uses the same **450 m cruise altitude**, **38 m/s maximum cruise speed**, fixed virtual receiver corridor, link-budget and cylindrical obstacle model described in the paper.

## Complete final workflow

```matlab
runMonteCarlo_A1_A2
make_MCmedian_representative_figures
make_combined_fig1_for_paper
```

The figure driver calls the included `plotRouteAndProfiles_korean_obstacles`, its own local comparison plotting helpers, and the same current filter/reference modules. All functions required for this final workflow are included in the selected 19-source snapshot. Optional combined image composition uses Image Processing Toolbox.

An earlier `compare_Baseline_Proposed_terminal` file has a mismatched saved filename and refers to a missing `plotRouteAndProfiles_korean` helper. It is not required by the final Monte Carlo representative-figure workflow and was not promoted as the default script. The B0–B8 tuning experiments, older IMM implementations and early ICROS prototypes were inventoried and retained privately rather than mixed into the final implementation.

## Public inputs and reproducibility

The numbers above identify which original source produced the paper workflow; they are not a claim that the public synthetic demo reproduces the paper benchmark. Original local GIS inputs and precomputed result files are not redistributed. The repository includes clearly fabricated obstacle inputs and a tested `run_demo`. Full benchmark reproduction requires appropriately licensed original-equivalent inputs and recorded settings.
