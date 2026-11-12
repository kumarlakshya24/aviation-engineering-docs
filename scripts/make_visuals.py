"""Generate the README figures from the repository's own modules and synthetic inputs.

Run with `make visuals` or `uv run scripts/make_visuals.py`. Output goes to docs/assets/.
Seeds are fixed, so the PNGs are reproducible.
"""

from __future__ import annotations

import logging
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap

ROOT = Path(__file__).resolve().parent.parent
# Module directories use hyphens, so they cannot be imported as packages.
sys.path[:0] = [str(ROOT / "flight-controller"), str(ROOT / "battery-management")]

from bms_monitor import BMSMonitor, CellReading, CellState
from sensor_fusion import SensorFusionEKF, SensorReading

log = logging.getLogger("make_visuals")

ASSETS = ROOT / "docs" / "assets"
G = 9.80665
M_PER_DEG = 111319.0  # matches SensorFusionEKF._lla_to_ned

# Dark theme. Series colours validated for CVD separation and 3:1 contrast on #18181b.
SURFACE = "#18181b"
INK = "#fafafa"
MUTED = "#a1a1aa"
GRID = "#27272a"
BLUE = "#3987e5"
ORANGE = "#d95926"
STATUS = {"warning": "#fab219", "serious": "#ec835a", "critical": "#d03b3b"}
SEQUENTIAL = ["#0d366b", "#184f95", "#256abf", "#3987e5", "#6da7ec", "#9ec5f4", "#cde2fb"]


def _style(ax: plt.Axes) -> None:
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color("#52525b")
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def sensor_fusion_figure(path: Path) -> None:
    rng = np.random.default_rng(7)
    imu_hz, gps_every = 100, 50  # 100 Hz IMU, 2 Hz GPS
    t = np.arange(0, 60, 1 / imu_hz)

    # Truth: level flight north. Accelerate, cruise, decelerate.
    a_north = np.where(t < 10, 1.5, np.where(t > 45, -1.0, 0.0))
    v_north = np.cumsum(a_north) / imu_hz
    p_north = np.cumsum(v_north) / imu_hz

    accel_bias = np.array([0.03, -0.02, 0.01])
    ekf = SensorFusionEKF()
    fused, gps_t, gps_n = [], [], []
    for k, tk in enumerate(t):
        # Level z-down accelerometer reads specific force: a - g, i.e. -g on body z at rest.
        accel = np.array([a_north[k], 0.0, -G]) + accel_bias + rng.normal(0, 0.05, 3)
        reading = SensorReading(
            timestamp_us=round(tk * 1e6),
            accel_xyz=accel,
            gyro_xyz=rng.normal(0, 1e-4, 3),
        )
        ekf.predict(reading)
        if k % gps_every == 0:
            north = p_north[k] + rng.normal(0, 1.5)
            reading.gps_lla = np.array([north / M_PER_DEG, rng.normal(0, 1.5) / M_PER_DEG, 0.0])
            reading.gps_quality = 2
            ekf.update_gps(reading)
            gps_t.append(tk)
            gps_n.append(north)
        fused.append(ekf.state.position_ned[0])

    fused_arr = np.array(fused)
    gps_idx = np.arange(0, len(t), gps_every)
    gps_err = np.array(gps_n) - p_north[gps_idx]
    fused_err = fused_arr - p_north
    gps_rms = float(np.sqrt(np.mean(gps_err**2)))
    fused_rms = float(np.sqrt(np.mean(fused_err[imu_hz * 5 :] ** 2)))  # skip 5 s convergence
    log.info("sensor fusion: GPS RMS %.2f m, fused RMS %.2f m", gps_rms, fused_rms)

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(10, 6.2), sharex=True, gridspec_kw={"height_ratios": [2, 1]}
    )
    fig.patch.set_facecolor(SURFACE)
    for ax in (ax1, ax2):
        _style(ax)

    ax1.plot(t, p_north, color=MUTED, linewidth=1.5, linestyle="--", label="Truth")
    ax1.scatter(gps_t, gps_n, s=14, color=ORANGE, edgecolors=SURFACE, linewidths=0.8,
                label="Raw GPS (2 Hz, 1.5 m noise)", zorder=3)
    ax1.plot(t, fused_arr, color=BLUE, linewidth=2, label="EKF estimate (100 Hz)", zorder=4)
    ax1.set_ylabel("North position (m)", color=MUTED)
    ax1.legend(frameon=False, labelcolor=INK, fontsize=9, loc="upper left")

    ax2.axhline(0, color="#52525b", linewidth=1)
    ax2.scatter(gps_t, gps_err, s=12, color=ORANGE, edgecolors=SURFACE, linewidths=0.8,
                label=f"GPS error, RMS {gps_rms:.2f} m")
    ax2.plot(t, fused_err, color=BLUE, linewidth=2, label=f"EKF error, RMS {fused_rms:.2f} m")
    ax2.set_ylabel("Error (m)", color=MUTED)
    ax2.set_xlabel("Time (s)", color=MUTED)
    ax2.legend(frameon=False, labelcolor=INK, fontsize=9, loc="upper right", ncols=2)

    fig.suptitle("Sensor fusion: raw GPS vs EKF estimate (synthetic flight)",
                 color=INK, fontsize=13, x=0.06, ha="left")
    fig.tight_layout()
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)


def bms_figure(path: Path) -> None:
    rng = np.random.default_rng(11)
    modules, cells = 6, 66
    volts = rng.normal(3.86, 0.015, (modules, cells))
    temps = rng.normal(28.0, 1.5, (modules, cells))
    resist = rng.normal(24.0, 2.0, (modules, cells))

    # Injected anomalies so every state is visible.
    temps[4, 40:52] = rng.normal(47.0, 0.8, 12)  # hot spot near a coolant outlet
    volts[1, [5, 6]] = 2.96  # low cells
    resist[2, [20, 33, 47]] = 56.0  # aged cells
    volts[5, 12] = 2.74  # failed cell

    bms = BMSMonitor()
    states = np.zeros((modules, cells), dtype=int)
    for m in range(modules):
        for c in range(cells):
            states[m, c] = bms.update_cell(CellReading(
                module_id=m, cell_id=c, voltage=float(volts[m, c]),
                temperature=float(temps[m, c]), internal_resistance=float(resist[m, c]),
                cycle_count=420,
            ))
    status = bms.get_pack_status(current_draw=180.0)
    log.info("bms: pack %s, SOC %.0f%%, SOH %.0f%%", status.state.name,
             status.soc_percent, status.soh_percent)

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(10, 6.2), gridspec_kw={"height_ratios": [1.6, 1]}
    )
    fig.patch.set_facecolor(SURFACE)
    _style(ax2)
    ax1.set_facecolor(SURFACE)

    cmap = LinearSegmentedColormap.from_list("blue", SEQUENTIAL)
    shown = np.clip(volts, 3.80, 3.92)  # out-of-band cells are marked below
    im = ax1.imshow(shown, cmap=cmap, aspect="auto", interpolation="nearest")
    ax1.set_yticks(range(modules), [f"M{m + 1}" for m in range(modules)])
    ax1.set_xlabel("Cell index within module", color=MUTED)
    ax1.tick_params(colors=MUTED, labelsize=9, length=0)
    for spine in ax1.spines.values():
        spine.set_visible(False)
    cbar = fig.colorbar(im, ax=ax1, pad=0.01)
    cbar.set_label("Cell voltage (V)", color=MUTED)
    cbar.ax.tick_params(colors=MUTED, labelsize=8)
    cbar.outline.set_visible(False)

    markers = [
        (CellState.WARNING, "^", STATUS["warning"], "Warning"),
        (CellState.DEGRADED, "s", STATUS["serious"], "Degraded"),
        (CellState.FAULT, "X", STATUS["critical"], "Fault"),
    ]
    for state, marker, color, label in markers:
        m_idx, c_idx = np.nonzero(states == state)
        size = 110 if state == CellState.FAULT else 46
        ax1.scatter(c_idx, m_idx, marker=marker, s=size, color=color, edgecolors=INK,
                    linewidths=1.0, label=label)
    ax1.legend(frameon=False, labelcolor=INK, fontsize=9, ncols=3,
               loc="upper left", bbox_to_anchor=(0, 1.16))

    labels = ["Nominal", "Warning", "Degraded", "Fault"]
    counts = [int(np.sum(states == s)) for s in
              (CellState.NOMINAL, CellState.WARNING, CellState.DEGRADED, CellState.FAULT)]
    colors = ["#52525b", STATUS["warning"], STATUS["serious"], STATUS["critical"]]
    bars = ax2.barh(labels, counts, color=colors, height=0.6, edgecolor=SURFACE, linewidth=2)
    ax2.invert_yaxis()
    ax2.set_xlabel("Cells", color=MUTED)
    ax2.grid(axis="y", visible=False)
    for bar, n in zip(bars, counts, strict=True):
        ax2.text(bar.get_width() + 4, bar.get_y() + bar.get_height() / 2, str(n),
                 va="center", color=INK, fontsize=9)

    fig.suptitle(
        f"Battery cell health: {modules * cells} cells, pack state {status.state.name}, "
        f"SOC {status.soc_percent:.0f}%, SOH {status.soh_percent:.0f}% (synthetic)",
        color=INK, fontsize=13, x=0.06, ha="left",
    )
    fig.tight_layout()
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    ASSETS.mkdir(parents=True, exist_ok=True)
    sensor_fusion_figure(ASSETS / "sensor-fusion.png")
    bms_figure(ASSETS / "bms-cell-health.png")
    log.info("wrote %s", ", ".join(p.name for p in sorted(ASSETS.glob("*.png"))))


if __name__ == "__main__":
    main()
