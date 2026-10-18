"""Summarise and plot a replayed log (see replay.c), and check the mag axes.
    ./replay < walk.log > walk.csv && python3 plot_log.py walk.csv
Needs numpy and matplotlib (pip install numpy matplotlib)."""
import csv
import itertools
import sys
from collections import Counter

import numpy as np

rows = [r for r in csv.DictReader(open(sys.argv[1])) if float(r["amag"]) > 0]  # skip step warm-up
col = lambda k: np.array([float(r[k]) for r in rows])
dt = np.diff(col("t"))
t = np.r_[0, np.cumsum(np.where(dt < 0, 20, dt))] / 1000.0  # keeps counting through reboots
ev = [r["event"] for r in rows]

# ---- Steps: what happened to every peak->valley cycle ----------------------------
counts = Counter(e for e in ev if e)
print(f"{t[-1]:.1f} s, {int(col('steps')[-1])} steps, {int(col('dist_cm')[-1])} cm;  cycles: {dict(counts)}")

# ---- Mag axis check ---------------------------------------------------------------
# With the mag axes matching the accelerometer's, the dip angle is the same in every
# pose. Try all 48 axis swaps/flips on the still samples and rank them by dip spread.
a = np.c_[col("ax"), col("ay"), col("az")]
mc = np.c_[col("mx"), col("my"), col("mz")]
mc = (mc - (mc.max(0) + mc.min(0)) / 2) / ((mc.max(0) - mc.min(0)) / 2)  # min/max cal, as on the board
still = abs(np.linalg.norm(a, axis=1) - 1000) < 50
down = -a[still] / np.linalg.norm(a[still], axis=1)[:, None]
ms = mc[still]


def dip(p, s):
    mm = ms[:, p] * s
    return np.degrees(np.arcsin(np.clip(np.sum(down * mm, 1) / np.linalg.norm(mm, axis=1), -1, 1)))


def name(p, s):
    return ", ".join(f"{'xyz'[i]}={'+-'[s[i] < 0]}m{'xyz'[p[i]]}" for i in range(3))


fits = []
for p in itertools.permutations(range(3)):
    for s in itertools.product((1, -1), repeat=3):
        d = dip(list(p), np.array(s))
        if d.mean() < 0:  # southern hemisphere: the field points up
            fits.append((d.std(), d.mean(), name(p, s)))
fits.sort()
d = dip([0, 1, 2], np.ones(3))
print(f"\nmag axis check on {still.sum()} still samples (want dip steady at about -63 deg):")
print(f"  as wired    {name((0, 1, 2), (1, 1, 1))}: dip {d.mean():6.1f} +/- {d.std():4.1f}")
for sd, mean, n in fits[:3]:
    print(f"  candidate   {n}: dip {mean:6.1f} +/- {sd:4.1f}")
if np.linalg.norm(down.mean(0)) > 0.9:
    print("  ! board barely tilted in this log: tumble it through all poses or this check means little")
print("  If a candidate beats 'as wired' by a lot, apply it to MAG_Value in readMag() (main.c).")

# ---- Gyro axis check --------------------------------------------------------------
# Turning the board turns the field it sees: dm/dt = -w x m. Rank all 48 gyro axis
# swaps/flips by how well they explain the mag vector's motion while turning.
w = np.c_[col("gx"), col("gy"), col("gz")] * np.pi / 180000  # mdps -> rad/s
turning = np.linalg.norm(w, axis=1) > 0.5                    # > ~30 dps
if turning.sum() > 50:
    dm = (np.gradient(mc, axis=0) / np.gradient(t)[:, None])[turning]
    fits = []
    for p in itertools.permutations(range(3)):
        for s in itertools.product((1, -1), repeat=3):
            res = dm + np.cross(w[turning][:, list(p)] * s, mc[turning])
            fits.append((np.median(np.linalg.norm(res, 1)) / np.median(np.linalg.norm(dm, 1)), name(p, s).replace("m", "g")))
    fits.sort()
    wired = [f for f in fits if f[1] == name((0, 1, 2), (1, 1, 1)).replace("m", "g")][0]
    print(f"\ngyro axis check on {turning.sum()} turning samples (want error well under 0.5):")
    print(f"  as wired    {wired[1]}: error {wired[0]:.2f}")
    for err, n in fits[:3]:
        print(f"  candidate   {n}: error {err:.2f}")
    print("  If a candidate beats 'as wired' by a lot, apply it to GYR_Value in readGyro() (main.c).")

# ---- Plots ----------------------------------------------------------------------------
import matplotlib.pyplot as plt  # noqa: E402  (text results above work without it)

fig, ax = plt.subplots(4, 1, sharex=True, figsize=(13, 10))
shade = lambda k, mask, c: ax[k].fill_between(t, 0, 1, where=mask, color=c, transform=ax[k].get_xaxis_transform(), lw=0)

shade(0, col("moving") == 0, "0.92")
ax[0].plot(t, col("amag"), color="0.75", lw=0.5, label="|a|")
ax[0].plot(t, col("filt"), "k", lw=1, label="filtered")
ax[0].plot(t, col("mean"), color="tab:gray", lw=1, label="mean")
ax[0].plot(t, col("thr_hi"), "--", color="tab:gray", lw=0.8, label="thresholds")
ax[0].plot(t, col("thr_lo"), "--", color="tab:gray", lw=0.8)
colours = {"step": "tab:green", "held": "tab:blue", "still": "tab:brown", "small": "tab:orange",
           "fast": "tab:red", "timeout": "tab:purple"}
filt = col("filt")
for e, c in colours.items():
    i = [k for k, x in enumerate(ev) if x == e]
    if i:
        ax[0].plot(t[i], filt[i], "o", color=c, ms=5, label=f"{e} ({len(i)})")
ax[0].set_ylabel("mg")
ax[0].set_title("steps: grey background = variance gate shut; dots = end of each peak->valley cycle")
ax[0].legend(loc="upper right", fontsize=8, ncol=4)

shade(1, col("cal") == 0, "0.92")
rej = col("rej") == 1
ax[1].plot(t, col("mag_heading"), ".", ms=1, color="0.6", label="mag alone")
ax[1].plot(t, col("heading"), ".", ms=1.5, color="tab:blue", label="heading (gyro + mag)")
ax[1].plot(t[rej], col("heading")[rej], ".", ms=2, color="tab:red", label="mag rejected (gyro only)")
ax[1].set_ylim(0, 360)
ax[1].set_ylabel("heading (deg)")
ax[1].set_title("heading: grey background = not calibrated yet (raw mag used)")
ax[1].legend(loc="upper right", fontsize=8)

ax[2].plot(t, col("dip"), lw=0.8)
ax[2].axhline(-63, color="tab:green", ls="--", lw=0.8)
ax[2].set_ylabel("dip (deg)")
ax[2].set_title("dip: should sit on -63 whatever the tilt once calibrated; swings = axis mismatch or bad cal")

ax[3].plot(t, col("mnorm"), lw=0.8, label="|m| calibrated")
ax[3].plot(t, col("radius"), color="k", lw=0.8, label="R (expected |m|)")
ax[3].fill_between(t, 0.8 * col("radius"), 1.2 * col("radius"), color="tab:green", alpha=0.1, label="gate +/-20%")
ax[3].plot(t, col("span"), color="tab:orange", lw=0.8, label="smallest axis span")
ax[3].axhline(800, color="tab:orange", ls="--", lw=0.8)
ax[3].set_ylabel("mG")
ax[3].set_xlabel("time (s)")
ax[3].set_title("mag: |m| outside the band = heading held; calibration starts when span reaches 800")
ax[3].legend(loc="upper right", fontsize=8, ncol=4)

fig.tight_layout()
plt.show()
