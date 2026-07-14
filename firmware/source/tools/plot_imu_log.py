#!/usr/bin/env python3
"""Plot IMU log: temp, heading, yaw-drift-rate, heater state → PNG."""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


DEFAULT_INPUT  = Path("~/imu_log.txt")
DEFAULT_OUTPUT = Path("~/imu_plot.png")


@dataclass(frozen=True)
class Sample:
    tag:    str
    t:      float   # seconds since boot
    temp:   float   # °C
    hdg:    float   # compass heading, deg
    drift:  float   # yaw drift rate, dps
    heater: int     # 0/1


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Plot IMU log (tag t temp hdg drift heater) to PNG."
    )
    p.add_argument("--input",  type=Path, default=DEFAULT_INPUT)
    p.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return p.parse_args()


def load(path: Path) -> list[Sample]:
    path = path.expanduser()
    if not path.exists():
        raise ValueError(f"file not found: {path}")
    samples: list[Sample] = []
    with path.open(encoding="utf-8") as fh:
        for n, raw in enumerate(fh, 1):
            parts = raw.strip().split()
            if not parts:
                continue
            if len(parts) != 6:
                raise ValueError(
                    f"line {n}: expected 6 columns, got {len(parts)}: {raw!r}"
                )
            tag, t, temp, hdg, drift, heater = parts
            samples.append(Sample(tag, float(t), float(temp),
                                  float(hdg), float(drift), int(heater)))
    if not samples:
        raise ValueError(f"no samples in {path}")
    return samples


def shade_heater(ax, t_hr: list[float], heater: list[int]) -> None:
    in_span = False
    span_start = 0.0
    for t, h in zip(t_hr, heater):
        if h and not in_span:
            span_start = t
            in_span = True
        elif not h and in_span:
            ax.axvspan(span_start, t, color="#fed7aa", alpha=0.5, linewidth=0)
            in_span = False
    if in_span:
        ax.axvspan(span_start, t_hr[-1], color="#fed7aa", alpha=0.5, linewidth=0)


def main() -> int:
    args = parse_args()
    try:
        samples = load(args.input)
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    t_hr   = [s.t / 3600        for s in samples]
    temp   = [s.temp             for s in samples]
    hdg    = [s.hdg              for s in samples]
    drift  = [s.drift * 3600     for s in samples]   # dps → °/h
    heater = [s.heater           for s in samples]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    fig.suptitle(
        f"IMU temperature / heading / drift  "
        f"({len(samples)} samples,  t: {min(t_hr):.2f}–{max(t_hr):.2f} h)",
        fontsize=13, fontweight="bold",
    )

    # ── top: temperature + heater shading ──────────────────────────────────
    shade_heater(ax1, t_hr, heater)
    ax1.plot(t_hr, temp, color="#dc2626", linewidth=1.6, label="temp °C")
    ax1.set_ylabel("temp °C", color="#dc2626")
    ax1.tick_params(axis="y", labelcolor="#dc2626")
    ax1.grid(True, linestyle="--", alpha=0.4)
    heater_patch = mpatches.Patch(color="#fed7aa", alpha=0.8, label="heater on")
    ax1.legend(handles=[ax1.lines[0], heater_patch], loc="upper right", fontsize=9)

    # ── bottom: heading (left) + drift (right) + heater shading ────────────
    shade_heater(ax2, t_hr, heater)
    ax2.plot(t_hr, hdg, color="#2563eb", linewidth=1.6, label="heading °")
    ax2.set_ylabel("heading °", color="#2563eb")
    ax2.tick_params(axis="y", labelcolor="#2563eb")
    ax2.set_xlabel("time (hours)")
    ax2.grid(True, linestyle="--", alpha=0.4)

    ax3 = ax2.twinx()
    ax3.plot(t_hr, drift, color="#16a34a", linewidth=1.2,
             linestyle="--", alpha=0.85, label="drift °/h")
    ax3.set_ylabel("yaw drift °/h", color="#16a34a")
    ax3.tick_params(axis="y", labelcolor="#16a34a")
    ax3.axhline(0, color="#16a34a", linewidth=0.6, linestyle=":")

    lines  = [l for l in ax2.lines + ax3.lines if not l.get_label().startswith("_")]
    labels = [l.get_label() for l in lines]
    ax2.legend(lines, labels, loc="upper right", fontsize=9)

    plt.tight_layout()
    out = args.output.expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
