"""
record.py  —  Serial recorder for Experiment 1 validation
---------------------------------------------------------------
Connects to the Hapkit board, lets you send commands interactively
(L, G, R, K30.0, B1.5, etc.) and saves each run to its own CSV file.

Usage:
    python record.py              # auto-detects port
    python record.py COM3         # Windows explicit port
    python record.py /dev/ttyUSB0 # Linux explicit port

Controls (type and press Enter):
    L        toggle live position monitor
    G        start run — opens a NEW file for this run
    S        stop
    R        reset calibration
    K30.0    set K = 30 N/m  (any parameter command works)
    quit     exit the script

Output:
    One file per run:  run_YYYYMMDD_HHMMSS.csv
    Files are created in the same folder as this script.
"""

import sys
import serial
import serial.tools.list_ports
import threading
from datetime import datetime

BAUD = 500000

# ── Auto-detect port ──────────────────────────────────────────────────────
def find_port():
    if len(sys.argv) > 1:
        return sys.argv[1]
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description + (p.manufacturer or "")).lower()
        if any(k in desc for k in ["arduino", "ch340", "ftdi", "uart", "usb serial"]):
            print(f"Auto-detected: {p.device}  ({p.description})")
            return p.device
    if ports:
        print(f"Using first available port: {ports[0].device}")
        return ports[0].device
    print("ERROR: No serial port found. Pass the port as argument.")
    sys.exit(1)


# ── Shared file handle ────────────────────────────────────────────────────
# A list is used as a mutable container so the read thread always writes
# to whatever file the main loop has most recently opened.
# current_file[0] is either an open file object or None.
current_file = [None]
stop_flag    = threading.Event()


# ── Serial read thread ────────────────────────────────────────────────────
def read_thread(ser):
    while not stop_flag.is_set():
        try:
            raw  = ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").rstrip()
            print(line)                     # always show in terminal

            # Write to the current file if one is open
            f = current_file[0]
            if f is not None and line:
                f.write(line + "\n")
                f.flush()

        except serial.SerialException:
            break
        except Exception:
            pass


def open_new_file():
    """Close the current file (if any) and open a fresh one."""
    close_current_file()
    ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"run_{ts}.csv"
    f        = open(filename, "w")
    f.write("# Experiment 1 Validation — Haptic Lab System\n")
    f.write(f"# Recorded: {datetime.now().isoformat()}\n")
    f.write("# Columns: t_ms, x_mm, dx_mms, forceN, motorEnabled\n")
    current_file[0] = f
    print(f">>> Saving this run to: {filename}")
    return filename


def close_current_file():
    """Flush and close the current file cleanly."""
    f = current_file[0]
    if f is not None:
        try:
            f.flush()
            f.close()
        except Exception:
            pass
        current_file[0] = None


# ── Main ──────────────────────────────────────────────────────────────────
def main():
    port = find_port()

    print(f"Opening {port} at {BAUD} baud ...")
    print("-" * 52)
    print("Commands (type and press Enter):")
    print("  L        live position monitor (toggle)")
    print("  G        start run  <- opens a new CSV file")
    print("  S        stop")
    print("  R        reset calibration")
    print("  K<val>   e.g. K30.0")
    print("  B<val>   e.g. B1.5")
    print("  quit     exit")
    print("-" * 52)

    with serial.Serial(port, BAUD, timeout=0.1) as ser:

        # Start background reader — it writes to whatever current_file[0] is
        t = threading.Thread(target=read_thread, args=(ser,), daemon=True)
        t.start()

        while True:
            try:
                cmd = input()
            except (EOFError, KeyboardInterrupt):
                break

            cmd = cmd.strip()
            if cmd.lower() == "quit":
                break

            # Intercept G so we can open a new file BEFORE sending to board
            if cmd.upper() == "G":
                open_new_file()

            ser.write((cmd + "\n").encode("ascii"))

    close_current_file()
    stop_flag.set()
    print("\nDone.")


if __name__ == "__main__":
    main()
