# UAM Surveillance & Positioning

MATLAB simulation of MLAT/TDOA tracking for an urban air mobility corridor, with a CV-EKF baseline and soft flight-plan priors under reduced receiver availability.

## Project context and my role

I am **LEE SANGYEOP (이상엽)**, a student researcher in the Autonomous Systems and Optimization Laboratory (ASOL), Korea Aerospace University. In the **2025 UAM Olympiad**, I served as deputy team leader of **AirWave**, contributing to the research direction, team coordination and simulation. The team received the Minister of Land, Infrastructure and Transport Award in radio environment analysis.

This source snapshot is the **later MATLAB MLAT tracking research**, associated with my first-author ICROS 2026 paper/poster, *Robust UAM Tracking Using Prior Flight Information under Reduced MLAT Receiver Availability*. It is not represented as the exact original Olympiad submission and does not implement the full Olympiad MLAT/5G hybrid system.

[Public Olympiad project overview](https://large-eucalyptus-ee4.notion.site/UAM-30982be26a1180f9a53ec453d3e7580c)

## What is included

- Three-dimensional takeoff, cruise, approach and descent trajectories.
- Virtual receiver sites, RF link budget, LOS/NLOS obstacle checks and TDOA noise.
- Baseline CV-EKF, A1 phase-aware process noise and nominal flight-plan priors, and A2 conservative three-receiver updates.
- Monte Carlo perturbation of measurement noise, missed detections, landing position and planned-versus-actual timing.
- Plotting helpers for the associated simulation workflow.

The Monte Carlo revision was selected because it explicitly separates the perturbed actual trajectory from the nominal planned reference. Intermediate patches, duplicate copies and older IMM experiments are not mixed into this version.

## Quick start

Open this folder in MATLAB and run:

```matlab
run_demo
```

The demo uses a nominal reference and a perturbed actual trajectory, and compares all three filter configurations with identical per-step measurement seeds. The included obstacle files are **fabricated demonstration data**. Their legacy filenames are retained for compatibility; they contain neither the original GIS dataset nor measured flight data. Printed errors are demonstration results, not the paper or competition benchmark.

For the original 50-trial evaluation workflow using the same synthetic inputs:

```matlab
runMonteCarlo_A1_A2
make_MCmedian_representative_figures
% Optional image composition; requires Image Processing Toolbox:
make_combined_fig1_for_paper
```

Run these commands from the repository root. The Monte Carlo script writes CSV/PNG outputs to the current folder. Generated files are ignored by Git. To reproduce an original benchmark, separately obtain an appropriately licensed obstacle dataset and record its provenance and settings; that dataset is not redistributed here.

## Files

| Entry | Purpose |
|---|---|
| `run_demo.m` | Public demonstration and numerical smoke check |
| `config_uam.m` | Virtual corridor, receivers, RF parameters and filter options |
| `generateTruth3D.m` | Flight trajectory generator |
| `getTDOAMeasurementFlexible.m` | RF detection, TDOA noise and receiver dropout |
| `runSingleEKF.m` | Tracking with optional terminal priors |
| `buildPhasePrior6.m` | Soft nominal flight-plan observations |
| `runMonteCarlo_A1_A2.m` | Paired Monte Carlo comparison |
| `make_MCmedian_representative_figures.m` | Figures for a median A2 trial |

## Requirements and limitations

MATLAB **R2024b** is the local development installation. Percentiles use `prctile`; optional figure composition uses Image Processing Toolbox. The included `run_demo` was numerically checked in R2024b; see [validation record](VALIDATION.md). GNU Octave compatibility has not been verified. This is a research simulation, not an operational aircraft positioning system. Virtual site heights, the RF model and the simplified cylindrical obstacles are modeling assumptions. The helper `runSingleEKF` falls back to using truth as the prior if its fifth argument is omitted; for profile-mismatch studies always pass a separate nominal `priorRef`, as `run_demo` and the Monte Carlo script do.

한글 요약: UAM 올림피아드 AirWave 부팀장 경험에서 이어진 ICROS MLAT 추적 연구 코드입니다. 공개용 예제에는 직접 만든 합성 장애물만 포함했으며, 산학과제의 C++ 항적융합 소스·내부 데이터·미심사 원고는 포함하지 않았습니다.

## Attribution and publication scope

This portfolio presents my research implementation and analysis while preserving the distinction between my role and the team/advisor contribution. No Hanwha Systems or IB Leaders contract source, journal manuscript, raw internal reports, binaries or original third-party GIS records are included. No blanket open-source license is assigned to an unlicensed research snapshot.
