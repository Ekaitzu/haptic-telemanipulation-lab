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
├── simulation/          # MATLAB/Simulink simulation scripts
│   ├── exp1/            # Experiment 1 parameter sweep script
│   ├── exp2/            # Experiment 2 parameter sweep script
│   ├── exp3_stg1/       # Experiment 3 Stage 1 parameter sweep script
│   ├── exp3_stg2/       # Experiment 3 Stage 2 parameter sweep script
│   ├── models/          # Simulink model files (.slx)
│   └── results/         # Saved sweep results and figures
├── firmware/            # Embedded software (Arduino/ATmega328P)
│   ├── master/          # Master module firmware
│   ├── slave/           # Slave module firmware
│   └── shared/          # Shared libraries and headers
├── cad/                 # Mechanical design files
│   ├── hapkit_base/     # Modified Hapkit base and board mount
│   ├── control_module/  # Control module enclosure
│   └── assembly/        # Full assembly files
├── thesis/              # LaTeX source of the thesis
│   ├── chapters/        # Chapter .tex files
│   ├── images/          # Figures used in the thesis
│   └── appendices/      # Appendix .tex files
└── docs/                # Additional documentation
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
- Parallel Computing Toolbox (for Experiment 3 sweeps)

### Firmware
- Arduino IDE or PlatformIO
- ATmega328P toolchain

## Getting Started

### Running the simulations

1. Open MATLAB and navigate to the `simulation/` folder.
2. Open the desired Simulink model from `simulation/models/`.
3. Run the corresponding sweep script, e.g.:
   ```matlab
   run('simulation/exp1/exp1_sweep_final.m')
   ```
4. Results are saved to `simulation/results/` and figures are generated automatically.

> **Note:** Experiment 3 sweeps can take several hours. See the thesis appendix for timing estimates and configuration details.

### Flashing the firmware

1. Connect the Hapkit board via USB.
2. Open the master or slave firmware project.
3. Compile and upload using Arduino IDE or PlatformIO.
4. Connect the control module to the master via DB15.

## Simulation Overview

The simulation framework uses automated parameter-space sweeps to classify the system response as *decaying*, *near-critical*, or *divergent* across the adjustable control parameters. The classification is based on time-domain indicators including envelope ratio, settling time, and energy generation. Full details of the methodology and thresholds are documented in Chapter 5 and the appendices of the thesis.

| Experiment | Swept Parameters | Grid | Horizon |
|---|---|---|---|
| Experiment 1 | K, B | 150 × 150 | 3 s |
| Experiment 2 | K, B | 50 × 50 | 6 s |
| Experiment 3 Stg 1 | K, B, Td | 50 × 50 × 5 | 12 s |
| Experiment 3 Stg 2 | K, B, Td | 50 × 50 × 5 | 6 s |

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
  url         = {https://github.com/ekaitzuria/haptic-telemanipulation-lab}
}
```

## License

This project is released under the [MIT License](LICENSE).

## Acknowledgements

- Supervisor: Prof. Dr.-Ing. Stefan Friedrich
- Second Examiner: Prof. Dr. Tobias Kaupp
- Gerald Barthelmes (component procurement and 3D printing support)
- Original Hapkit platform: [Stanford University](https://hapkit.stanford.edu/)
