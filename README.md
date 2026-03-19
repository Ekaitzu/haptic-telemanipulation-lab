# Haptic and Telemanipulation Lab System

**Bachelor's Thesis — Ekaitz Uria**  
Technical University of Applied Sciences Würzburg-Schweinfurt (THWS)  
Faculty of Electrical Engineering — Bachelor of Robotics  
March 2026

## Overview

This repository contains the complete source material for the bachelor's thesis *"Development of a Haptic and Telemanipulation Lab System"*. The project develops a compact, low-cost educational laboratory platform for one-degree-of-freedom bilateral teleoperation, based on two modified [Hapkit](https://hapkit.stanford.edu/) haptic modules and a dedicated control subsystem.

Three experimental modes are implemented:

- **Experiment 1 — Local Virtual Environment:** Single-device spring–damper rendering with optional virtual wall.
- **Experiment 2 — Instant Bilateral Teleoperation:** Two-channel position–position coupling without communication delay.
- **Experiment 3 — Delayed Bilateral Teleoperation:** Same bilateral architecture with configurable transport delay.

## Repository Structure

```
├── firmware/                        # Embedded software (ATmega328P)
│   ├── main_master/                 # Master module firmware
│   │   └── main_master.ino
│   └── main_slave/                  # Slave module firmware
│       └── main_slave.ino
├── Simulation/                      # MATLAB/Simulink simulation
│   ├── Experiment 1/                # Local virtual environment
│   │   ├── exp1_local_rendering_stg1.slx
│   │   ├── exp1_local_rendering_stg2.slx
│   │   └── exp1_sweep_final.m
│   ├── Experiment 2/                # Instant bilateral teleoperation
│   │   ├── exp2_bilateral_nodelay_stg1.slx
│   │   ├── exp2_bilateral_nodelay_stg2.slx
│   │   └── exp2_sweep_final.m
│   ├── Experiment 3/                # Delayed bilateral teleoperation
│   │   ├── exp3_bilateral_delay_stg1.slx
│   │   ├── exp3_bilateral_delay_stg2.slx
│   │   ├── sweep_exp3_KB_Td_stg1_clean.m
│   │   └── sweep_exp3_KB_Td_stg2_clean.m
│   └── Validation/                  # Hardware validation scripts
│       └── Experiment 1/
│           ├── exp1_validation/
│           │   └── exp1_validation.ino
│           ├── plot_exp1.py
│           └── record.py
├── CITATION.cff
├── LICENSE
└── README.md
```

## Hardware Requirements

- 2× Modified Hapkit haptic modules ([original Hapkit project](https://hapkit.stanford.edu/))
- 2× Portescap 22N28 210E brushed DC motor
- 2× Hapkit board (ATmega328P-based)
- 1× Control module (OLED display, buttons, potentiometer, LEDs)
- 2× External 12 V / 1 A power supply
- DB15 D-sub connectors and cabling

## Software Requirements

### Simulation
- MATLAB R2023b or later
- Simulink
- Parallel Computing Toolbox (required for Experiment 3 sweeps)

### Firmware
- Arduino IDE or PlatformIO
- ATmega328P toolchain

### Validation
- Python 3 with `pyserial` and `matplotlib` (for hardware data recording and plotting)

## Getting Started

### Running the simulations

1. Open MATLAB and navigate to the desired experiment folder inside `Simulation/`.
2. Open the corresponding Simulink model (Stage 1 or Stage 2).
3. Run the sweep script from MATLAB, e.g.:
   ```matlab
   cd('Simulation/Experiment 1')
   run('exp1_sweep_final.m')
   ```
4. Figures and results are saved automatically in the script directory.

> **Note:** Experiment 3 sweeps use parallel workers and can take several hours. Adjust `cfg.nWorkers` in the script to match your machine. See the thesis appendix for timing estimates and full configuration details.

### Flashing the firmware

1. Connect the master Hapkit board via USB.
2. Open `firmware/main_master/main_master.ino` in Arduino IDE.
3. Compile and upload.
4. Repeat with `firmware/main_slave/main_slave.ino` for the slave board.
5. Connect the control module to the master via DB15 and power both modules.

### Hardware validation (Experiment 1)

1. Flash `Simulation/Validation/Experiment 1/exp1_validation/exp1_validation.ino` to the master board.
2. Run `record.py` to capture serial data during a release-response test.
3. Run `plot_exp1.py` to generate the hardware vs. simulation comparison plots.

## Simulation Overview

The simulation framework uses automated parameter-space sweeps to classify the system response as *decaying*, *near-critical*, or *divergent* across the adjustable control parameters. The classification is based on time-domain indicators including envelope ratio, settling time, and energy generation. Full details of the methodology and thresholds are documented in Chapter 5 and the appendices of the thesis.

| Experiment | Swept Parameters | Grid | Horizon |
|---|---|---|---|
| Experiment 1 | K, B | 150 × 150 | 3 s |
| Experiment 2 | K, B | 50 × 50 | 6 s |
| Experiment 3 Stage 1 | K, B, Td | 50 × 50 × 5 | 12 s |
| Experiment 3 Stage 2 | K, B, Td | 50 × 50 × 5 | 6 s |

Each experiment includes two model stages:
- **Stage 1:** Nominal continuous-time controller with linearized plant (baseline).
- **Stage 2:** Sampled-data controller realization matching the implemented 1 kHz digital control loop.

## CAD Files

The mechanical parts were designed in Onshape and are publicly accessible:

- [Modified Hapkit Base and Board Mount](https://cad.onshape.com/documents/c8fc6e6196f04e75e543bbd4/w/1d80618b4131a3cad4d1418a/e/6caf85e75e85b5af1110aae1?renderMode=0&uiState=69bc2a7469daf43287e62295)
- [Control Module Enclosure](https://cad.onshape.com/documents/8d4e7897996822ba89b4aaa7/w/1423f98d0d7cbf0eb18f9043/e/56cc62e6e25019a27bdcc36d)

## Citation

If you use this work, please cite:

```bibtex
@thesis{uria2026haptic,
  author      = {Uria, Ekaitz},
  title       = {Development of a Haptic and Telemanipulation Lab System},
  type        = {Bachelor's Thesis},
  institution = {Technical University of Applied Sciences W{\"u}rzburg-Schweinfurt},
  year        = {2026},
  month       = {3},
  url         = {https://github.com/Ekaitzu/haptic-telemanipulation-lab}
}
```

## License

This project is released under the [MIT License](LICENSE).

## Acknowledgements

- Supervisor: Prof. Dr.-Ing. Stefan Friedrich
- Second Examiner: Prof. Dr. Tobias Kaupp
- Gerald Barthelmes (component procurement and 3D printing support)
- Original Hapkit platform: [Stanford University](https://hapkit.stanford.edu/)
