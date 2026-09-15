"""Run the standard scenarios, print metrics, and write a figure (or CSV).

    python -m gnssdenied.demo                     # PNG if matplotlib is installed, else CSV
    python -m gnssdenied.demo --out results --csv # also write CSV time series
    python -m gnssdenied.demo --seed 3 --duration 600
"""
from __future__ import annotations

import argparse
import csv
import os
from dataclasses import replace

from .metrics import Summary
from .sim import Config, standard_scenarios

PALETTE = {
    "page": "#F5F7FA", "surface": "#FFFFFF", "line": "#D8DEE6", "ink": "#0E1726", "ink2": "#3D4A5C",
    "ink3": "#5F6B7C", "navy": "#0B2545", "steel": "#3E5C76", "brass": "#8A6A1F", "amber": "#B7791F",
    "teal": "#2F6F73",
}


def print_report(sc: dict, spoof_start_s: float) -> None:
    print("Whole run (errors in metres; NEES should average about 2 for a consistent 2D filter)\n")
    print(Summary.header())
    for r in sc.values():
        print(r.summary().row())
    print(f"\nAfter spoofing starts at t = {spoof_start_s:.0f} s\n")
    print(Summary.header())
    for key in ("spoof_naive", "spoof_monitored"):
        print(sc[key].summary(start_s=spoof_start_s).row())
    g = sc["vpr_gated"]
    outliers = [x for x in g.vpr_log if x[4]]
    print(
        f"\nVPR gating: {len(outliers)} wrong matches injected, "
        f"{sum(1 for x in outliers if not x[3])} rejected; "
        f"{sum(1 for x in g.vpr_log if not x[4] and not x[3])} correct fixes rejected."
    )
    m = sc["spoof_monitored"]
    if m.detection_time_s is not None:
        print(f"Consistency monitor flagged GNSS at t = {m.detection_time_s:.0f} s ({m.detection_reason}).")
    else:
        print("Consistency monitor did not flag GNSS in this run.")


def write_csv(sc: dict, out: str) -> list:
    os.makedirs(out, exist_ok=True)
    paths = []
    p = os.path.join(out, "summary.csv")
    with open(p, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["scenario", "rmse_m", "cep50_m", "cep95_m", "max_m", "final_m", "mean_nees", "fixes_accepted", "fixes_rejected"])
        for r in sc.values():
            s = r.summary()
            w.writerow([s.name, f"{s.rmse_m:.2f}", f"{s.cep50_m:.2f}", f"{s.cep95_m:.2f}", f"{s.max_m:.2f}",
                        f"{s.final_m:.2f}", f"{s.mean_nees:.3f}", s.fixes_accepted, s.fixes_rejected])
    paths.append(p)
    p = os.path.join(out, "timeseries.csv")
    with open(p, "w", newline="") as fh:
        w = csv.writer(fh)
        keys = list(sc)
        w.writerow(["t_s", "truth_x", "truth_y"] + [f"{k}_{c}" for k in keys for c in ("x", "y", "err")])
        errs = {k: sc[k].errors() for k in keys}
        truth = sc[keys[0]].truth
        for i, tp in enumerate(truth):
            row = [f"{tp[0]:.1f}", f"{tp[1]:.2f}", f"{tp[2]:.2f}"]
            for k in keys:
                e = sc[k].est[i]
                row += [f"{e[0]:.2f}", f"{e[1]:.2f}", f"{errs[k][i]:.2f}"]
            w.writerow(row)
    paths.append(p)
    p = os.path.join(out, "fixes.csv")
    with open(p, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["scenario", "source", "t_s", "x", "y", "accepted", "truly_bad", "d2"])
        for k, r in sc.items():
            for src, log in (("vpr", r.vpr_log), ("gnss", r.gnss_log)):
                for t, x, y, acc, bad, d2 in log:
                    w.writerow([k, src, f"{t:.1f}", f"{x:.2f}", f"{y:.2f}", int(acc), int(bad), f"{d2:.3f}"])
    paths.append(p)
    return paths


def write_png(sc: dict, out: str, spoof_start_s: float) -> str:
    import logging

    import matplotlib

    matplotlib.use("Agg")
    logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)
    import matplotlib.pyplot as plt

    c = PALETTE
    plt.rcParams.update({
        "font.family": ["IBM Plex Sans", "DejaVu Sans", "Arial", "sans-serif"],
        "font.size": 10, "axes.edgecolor": c["line"], "axes.labelcolor": c["ink2"],
        "xtick.color": c["ink3"], "ytick.color": c["ink3"], "axes.titlesize": 11.5,
        "axes.titleweight": "semibold", "axes.titlecolor": c["ink"], "axes.titlelocation": "left",
        "legend.frameon": False, "legend.fontsize": 9,
    })
    fig, axes = plt.subplots(2, 2, figsize=(12, 8.6), facecolor=c["page"])
    fig.subplots_adjust(left=0.07, right=0.98, top=0.855, bottom=0.07, hspace=0.36, wspace=0.18)
    for ax in axes.flat:
        ax.set_facecolor(c["surface"])
        ax.grid(color="#E7EBF0", linewidth=0.8)
        ax.set_axisbelow(True)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)

    truth = sc["vio_only"].truth
    tx, ty = [p[1] for p in truth], [p[2] for p in truth]
    t = [p[0] for p in truth]
    xlim = (min(tx) - 300, max(tx) + 300)
    # roughly equal metres per pixel on the map panels, with room below the track for the legend
    ymid = 0.5 * (min(ty) + max(ty)) + 150
    yspan = 0.55 * (xlim[1] - xlim[0])
    ylim = (ymid - 0.72 * yspan, ymid + 0.28 * yspan)

    ax = axes[0, 0]
    g = sc["vpr_gated"]
    ax.plot(tx, ty, color=c["ink"], lw=1.2, label="truth")
    ax.plot(*zip(*sc["vio_only"].est), color=c["steel"], lw=1.2, ls="--", label="VIO only (drifts)")
    ax.plot(*zip(*g.est), color=c["navy"], lw=1.6, label="EKF, VIO + VPR, gated")
    acc = [(x[1], x[2]) for x in g.vpr_log if x[3]]
    rej = [(x[1], x[2]) for x in g.vpr_log if not x[3]]
    if acc:
        ax.scatter(*zip(*acc), s=10, color=c["teal"], zorder=3, label="VPR fix accepted")
    if rej:
        ax.scatter(*zip(*rej), s=34, marker="x", color=c["brass"], zorder=4, label=f"VPR fix rejected ({len(rej)})")
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_title("(a) Relative motion plus gated absolute fixes")
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.legend(loc="lower center", ncol=2)

    ax = axes[0, 1]
    for key, col, ls in (("vio_only", c["steel"], "--"), ("vpr_no_gate", c["amber"], "-"), ("vpr_gated", c["navy"], "-")):
        ax.plot(t, [max(e, 0.1) for e in sc[key].errors()], color=col, lw=1.4, ls=ls, label=sc[key].name)
    ax.set_yscale("log")
    ax.set_title("(b) Position error: gating is what makes fixes useful")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("error (m, log scale)")
    ax.legend(loc="lower right")

    ax = axes[1, 0]
    n, m = sc["spoof_naive"], sc["spoof_monitored"]
    ax.plot(tx, ty, color=c["ink"], lw=1.2, label="truth")
    sp = [(x[1], x[2]) for x in n.gnss_log if x[4]]
    if sp:
        ax.plot(*zip(*sp), color=c["brass"], lw=0.9, alpha=0.8, label="spoofed GNSS (carry-off)")
    ax.plot(*zip(*n.est), color=c["amber"], lw=1.6, label="EKF, gate only: captured")
    ax.plot(*zip(*m.est), color=c["navy"], lw=1.6, label="EKF + consistency monitor")
    if m.detection_time_s is not None:
        i = int(round(m.detection_time_s / m.cfg.dt_s))
        ax.scatter([m.est[i][0]], [m.est[i][1]], s=60, facecolor="none", edgecolor=c["teal"], lw=2, zorder=5,
                   label=f"GNSS flagged, t = {m.detection_time_s:.0f} s")
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_ylim(ylim[0], ylim[1] + 250)
    ax.set_title("(c) Spoofing: the gate alone is dragged along")
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.legend(loc="lower center", ncol=2)

    ax = axes[1, 1]
    ax.plot(t, n.errors(), color=c["amber"], lw=1.5, label=n.name)
    ax.plot(t, m.errors(), color=c["navy"], lw=1.5, label=m.name)
    ax.axvline(spoof_start_s, color=c["brass"], ls="--", lw=1)
    ax.text(spoof_start_s, ax.get_ylim()[1] * 0.97, " carry-off starts", color=c["brass"], va="top", fontsize=9)
    if m.detection_time_s is not None:
        ax.axvline(m.detection_time_s, color=c["teal"], lw=1.2)
        ax.text(m.detection_time_s, ax.get_ylim()[1] * 0.85, " flagged", color=c["teal"], va="top", fontsize=9)
    ax.set_title("(d) Error under spoofing")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("error (m)")
    ax.legend(loc="center left")

    fig.text(0.07, 0.955, "DARKANALYTICA  ·  SYNTHETIC SIMULATION, SEEDED", color=c["ink3"], fontsize=9,
             family=["IBM Plex Mono", "DejaVu Sans Mono", "monospace"])
    fig.text(0.07, 0.915, "GNSS-denied navigation: drift, absolute fixes, innovation gating and carry-off",
             color=c["ink"], fontsize=16, weight="semibold", family=["IBM Plex Sans", "DejaVu Sans", "Arial", "sans-serif"])
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, "gnssdenied_demo.png")
    fig.savefig(path, dpi=130, facecolor=c["page"])
    plt.close(fig)
    return path


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m gnssdenied.demo", description=__doc__.splitlines()[0])
    ap.add_argument("--out", default="examples/output", help="output directory")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--duration", type=float, default=None, help="seconds (default 900)")
    ap.add_argument("--csv", action="store_true", help="also write CSV files")
    ap.add_argument("--no-plot", action="store_true", help="skip the PNG even if matplotlib is available")
    a = ap.parse_args(argv)
    cfg = Config() if a.duration is None else replace(Config(), duration_s=a.duration)
    sc = standard_scenarios(cfg, seed=a.seed)
    print_report(sc, cfg.spoof_start_s)
    wrote = []
    want_csv = a.csv or a.no_plot
    if not a.no_plot:
        try:
            wrote.append(write_png(sc, a.out, cfg.spoof_start_s))
        except ImportError:
            print("\nmatplotlib is not installed: writing CSV instead (pip install matplotlib for the figure).")
            want_csv = True
    if want_csv:
        wrote += write_csv(sc, a.out)
    print("\nWrote:")
    for p in wrote:
        print("  " + p)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
