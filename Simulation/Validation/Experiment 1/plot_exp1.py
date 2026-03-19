"""
plot_exp1.py  —  Hardware vs Simulation comparison for Experiment 1
---------------------------------------------------------------
Reads a CSV file recorded by record.py, detects individual runs
(separated by the header line  t_ms,x_mm,...), and plots each one.

If you also have Simulink output exported as a CSV or .mat file,
it overlays the simulation curve on the same axes.

Usage:
    python plot_exp1.py run_20260315_143022.csv
    python plot_exp1.py run_20260315_143022.csv --sim sim_K30_B1p5.csv

Simulation CSV format (exported from Simulink via "To Workspace" + csvwrite,
or from the MATLAB command: writematrix([t, x], 'sim.csv')):
    time_s, x_m          (two columns, no header, SI units)

Output:
    - Interactive plot window
    - PNG saved as  plot_<inputfilename>.png
"""

import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from pathlib import Path


# ── Argument parsing ──────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument("csvfile",          help="Recorded CSV from record.py")
parser.add_argument("--sim", default=None,
                    help="Simulation CSV: two columns (time_s, x_m), no header")
parser.add_argument("--runs", default=None,
                    help="Which runs to plot, e.g. --runs 1,2  (default: all)")
args = parser.parse_args()

csv_path = Path(args.csvfile)
if not csv_path.exists():
    print(f"ERROR: file not found: {csv_path}")
    sys.exit(1)


# ── Parse CSV — split into individual runs ────────────────────────────────
# A new run starts whenever the column-header line  t_ms,x_mm,...  appears.
# The K/B label is taken from the  # K=...  comment that the board prints
# just before the header.
runs      = []     # list of dicts, one per run
cur_rows  = []
cur_label = "Run 1"
pending_label = None   # label seen in comments, waiting for header line

with open(csv_path) as f:
    for line in f:
        line = line.strip()
        if not line:
            continue

        # Comment lines — capture the parameter label if present
        if line.startswith("#"):
            if line.startswith("# K="):
                pending_label = line[2:].strip()
            continue

        # Column-header line → start of a new run
        if line.startswith("t_ms"):
            if cur_rows:                        # save the previous run first
                runs.append({"label": cur_label, "rows": cur_rows})
                cur_rows = []
            cur_label     = pending_label if pending_label else f"Run {len(runs)+1}"
            pending_label = None
            continue

        # Data row
        parts = line.split(",")
        if len(parts) >= 2:
            try:
                cur_rows.append([float(p) for p in parts])
            except ValueError:
                pass

# Don't forget the last run
if cur_rows:
    runs.append({"label": cur_label, "rows": cur_rows})

if not runs:
    print("ERROR: No data runs found in the file.")
    sys.exit(1)

print(f"Found {len(runs)} run(s) in {csv_path.name}")


# ── Filter which runs to plot ──────────────────────────────────────────────
if args.runs:
    indices = [int(x)-1 for x in args.runs.split(",")]
    runs = [runs[i] for i in indices if i < len(runs)]


# ── Load simulation data (optional) ───────────────────────────────────────
sim_t = None
sim_x = None
if args.sim:
    sim_path = Path(args.sim)
    if sim_path.exists():
        sim_data = np.loadtxt(sim_path, delimiter=",")
        sim_t = sim_data[:, 0] * 1000.0   # s → ms
        sim_x = sim_data[:, 1] * 1000.0   # m → mm
        print(f"Loaded simulation: {sim_path.name}  ({len(sim_t)} points)")
    else:
        print(f"WARNING: sim file not found: {sim_path}")


# ── Plot ───────────────────────────────────────────────────────────────────
N = len(runs)
fig, axes = plt.subplots(N, 1, figsize=(10, 4 * N), squeeze=False)
fig.suptitle("Experiment 1 — Local Virtual Environment\nHardware vs Simulation",
             fontsize=13, fontweight="bold", y=1.01)

# Colour cycle for hardware runs
hw_colors = ["#2E75B6", "#C55A11", "#375623", "#7030A0"]

for i, run in enumerate(runs):
    ax = axes[i][0]
    data = np.array(run["rows"])

    t_ms = data[:, 0]
    x_mm = data[:, 1]

    # ── Hardware curve ────────────────────────────────────────────────────
    color = hw_colors[i % len(hw_colors)]
    ax.plot(t_ms, x_mm,
            color=color, linewidth=1.8,
            label=f"Hardware  ({run['label']})")

    # Mark the actual initial condition (first sample)
    x0_hw = x_mm[0]
    ax.axhline(x0_hw, color=color, linewidth=0.6, linestyle=":", alpha=0.5)
    ax.annotate(f"x₀ = {x0_hw:.2f} mm",
                xy=(t_ms[0], x0_hw),
                xytext=(t_ms[-1]*0.05, x0_hw + 0.3),
                fontsize=8, color=color)

    # ── Simulation curve ──────────────────────────────────────────────────
    if sim_t is not None:
        # Shift simulation time to align with hardware t=0
        ax.plot(sim_t, sim_x,
                color="black", linewidth=1.5, linestyle="--",
                label="Simulation (Simulink Stage 2)")

    # ── Rest position reference line ──────────────────────────────────────
    ax.axhline(0, color="gray", linewidth=0.8, linestyle="-", alpha=0.4,
               label="Rest position (0 mm)")

    # ── Formatting ────────────────────────────────────────────────────────
    ax.set_xlabel("Time [ms]", fontsize=10)
    ax.set_ylabel("Handle position [mm]", fontsize=10)
    ax.set_title(run["label"], fontsize=10, pad=6)
    ax.legend(fontsize=9, loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.xaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())

    # Keep x-axis starting at 0
    ax.set_xlim(left=0)

plt.tight_layout()

# ── Save PNG then show ────────────────────────────────────────────────────
out_png = csv_path.stem + "_plot.png"
plt.savefig(out_png, dpi=150, bbox_inches="tight")
print(f"Plot saved: {out_png}")

# show() can block forever on some backends — use a timed window instead.
# The plot stays open for 30 seconds then the script exits cleanly.
# Close the window manually at any time to exit immediately.
try:
    plt.show(block=False)
    plt.pause(30)
except Exception:
    pass
finally:
    plt.close("all")
    print("Done.")
