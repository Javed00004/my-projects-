"""Colab-ready 2D wall-climbing robot animation (kinematic model)."""

import math
import shutil

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle
try:
    from IPython.display import Video, display
except ImportError:  # Headless local rendering; Colab includes IPython.
    Video = display = None

# Change these four settings to explore different motions.
DURATION = 6.0       # seconds
FPS = 24
CLIMB_SPEED = 0.28   # wall units per second
GAIT_PERIOD = 1.5    # seconds per step cycle
OUTPUT = "wall_climbing_robot.mp4"

if shutil.which("ffmpeg") is None:
    raise RuntimeError("FFmpeg is needed. In Colab run: !apt-get -qq install ffmpeg")

plt.rcParams["font.family"] = "DejaVu Sans"
fig, ax = plt.subplots(figsize=(10, 7), dpi=100)
fig.patch.set_facecolor("#08131c")
ax.set_facecolor("#08131c")
ax.set_xlim(-2.0, 2.0)
ax.set_ylim(0.0, 3.6)
ax.set_aspect("equal")
ax.axis("off")
fig.subplots_adjust(left=0.02, right=0.98, bottom=0.02, top=0.94)

# Static brick wall, floor line and progress ruler.
ax.add_patch(Rectangle((-1.10, 0), 2.20, 3.6, color="#263b46", zorder=0))
ax.add_patch(Rectangle((-1.10, 0), 0.055, 3.6, color="#56717b", zorder=1))
for row, y in enumerate(np.arange(0.12, 3.6, 0.28)):
    ax.plot([-1.05, 1.10], [y, y], color="#3d5560", lw=1.1, zorder=1)
    offset = 0.0 if row % 2 else 0.23
    for x in np.arange(-1.05 + offset, 1.10, 0.46):
        ax.plot([x, x], [y, min(y + 0.28, 3.6)], color="#3d5560", lw=1, zorder=1)
ax.plot([-1.12, 1.13], [0.08, 0.08], color="#91aeb5", lw=2, zorder=2)
ax.plot([1.39, 1.39], [0.48, 3.05], color="#415864", lw=5, zorder=1)
for y in np.arange(0.5, 3.1, 0.5):
    ax.plot([1.34, 1.46], [y, y], color="#7694a0", lw=1, zorder=2)

fig.text(0.5, 0.958, "WALL-CLIMBING ROBOT", ha="center", va="top",
         fontsize=21, fontweight="bold", color="#f4f6f6")
fig.text(0.5, 0.917, "FOUR-PAD ALTERNATING GAIT  •  2D KINEMATIC MODEL",
         ha="center", va="top", fontsize=9, color="#8babb5")
ax.text(-1.89, 3.00, "MISSION", color="#6bdac9", fontsize=10,
        fontweight="bold", zorder=5)
ax.text(-1.89, 2.78, "VERTICAL\nASCENT", color="#e7eff0", fontsize=14,
        fontweight="bold", linespacing=1.35, zorder=5)
ax.text(-1.89, 2.38, "Green = sealed\nOrange = moving", color="#adc1c6",
        fontsize=9, linespacing=1.5, zorder=5)
ax.text(1.51, 3.12, "HEIGHT", color="#a8bfc4", fontsize=9, zorder=5)

moving_artists = []


def patch(shape):
    ax.add_patch(shape)
    moving_artists.append(shape)


def line(xs, ys, **style):
    moving_artists.extend(ax.plot(xs, ys, **style))


def label(x, y, value, **style):
    moving_artists.append(ax.text(x, y, value, **style))


def pad_position(t, side, level, offset):
    """Feet stay fixed on the wall during grip and advance during swing."""
    phase = (t / GAIT_PERIOD + offset) % 1.0
    stance_fraction = 0.72
    advance = CLIMB_SPEED * GAIT_PERIOD
    base_y = 0.29 if level == "upper" else -0.29
    top = base_y + 0.5 * stance_fraction * advance
    if phase < stance_fraction:
        return side * 0.75, top - advance * phase, True
    q = (phase - stance_fraction) / (1.0 - stance_fraction)
    smooth = q * q * (3.0 - 2.0 * q)
    x = side * (0.75 + 0.06 * math.sin(math.pi * q))
    y = top - stance_fraction * advance + stance_fraction * advance * smooth
    return x, y + 0.055 * math.sin(math.pi * q), False


def draw_frame(t):
    for artist in moving_artists:
        artist.remove()
    moving_artists.clear()

    body_y = 0.72 + CLIMB_SPEED * t
    # Diagonal pairs have opposite phase; at least two pads remain sealed.
    feet = [(-1, "upper", 0.0), (1, "upper", 0.5),
            (-1, "lower", 0.5), (1, "lower", 0.0)]
    gripped = 0
    for index, (side, level, offset) in enumerate(feet, start=1):
        x, local_y, sealed = pad_position(t, side, level, offset)
        y = body_y + local_y
        gripped += int(sealed)
        shoulder_x = side * 0.25
        shoulder_y = body_y + (0.19 if level == "upper" else -0.19)
        elbow_x = side * 0.52
        elbow_y = (shoulder_y + y) / 2 + (0.06 if level == "upper" else -0.06)

        line([shoulder_x, elbow_x, x], [shoulder_y, elbow_y, y],
             color="#080f16", lw=12, solid_capstyle="round", zorder=4)
        line([shoulder_x, elbow_x, x], [shoulder_y, elbow_y, y],
             color="#8ab9bc" if sealed else "#dba574",
             lw=6, solid_capstyle="round", zorder=5)
        patch(Circle((elbow_x, elbow_y), 0.044, color="#d9e6e4", zorder=6))
        patch(Circle((x, y), 0.116, color="#07151b", zorder=6))
        patch(Circle((x, y), 0.085, facecolor="#114f4b" if sealed else "#64402f",
                     edgecolor="#54e1b5" if sealed else "#ffb068",
                     linewidth=2.5, zorder=7))
        patch(Circle((x, y), 0.027,
                     color="#9af9d2" if sealed else "#ffd0a2", zorder=8))
        label(x, y - 0.16, f"S{index}", color="#c7d5d8", fontsize=7,
              ha="center", va="top", zorder=9)

    # Main chassis and its sensor/status lights.
    patch(FancyBboxPatch((-0.30, body_y - 0.33), 0.60, 0.66,
                         boxstyle="round,pad=0.04,rounding_size=0.09",
                         facecolor="#112831", edgecolor="#b5d6d7",
                         linewidth=2.3, zorder=10))
    patch(FancyBboxPatch((-0.23, body_y - 0.22), 0.46, 0.42,
                         boxstyle="round,pad=0.02,rounding_size=0.06",
                         facecolor="#edae49", edgecolor="#ffdb86",
                         linewidth=2, zorder=11))
    patch(Circle((0, body_y + 0.07), 0.082, facecolor="#244f5b",
                 edgecolor="#c6eff0", linewidth=2, zorder=12))
    patch(Circle((0, body_y + 0.07), 0.028, color="#6ce5ec", zorder=13))
    label(0, body_y - 0.125, "WC-01", color="#263642", fontsize=10,
          fontweight="bold", ha="center", va="center", zorder=13)
    label(-0.19, body_y + 0.25, "●", color="#77f2ba", fontsize=12, zorder=13)
    label(0.04, body_y + 0.25, "●", color="#77f2ba", fontsize=12, zorder=13)

    # Frame counter and live progress display.
    height = CLIMB_SPEED * t
    line([1.39, 1.39], [0.48, min(body_y, 3.05)],
         color="#61ddbe", lw=5, zorder=3)
    label(1.52, 2.91, f"{height:.2f} m", color="#e3eff0", fontsize=12,
          fontweight="bold", zorder=5)
    label(-1.89, 1.91, f"GRIP  {gripped}/4", color="#7de5ba", fontsize=11,
          fontweight="bold", zorder=5)
    label(-1.89, 1.70, f"TIME  {t:04.1f} s", color="#c1d4d8", fontsize=10, zorder=5)


writer = FFMpegWriter(fps=FPS, metadata={"title": "Wall-climbing robot simulation"},
                      bitrate=1800)
with writer.saving(fig, OUTPUT, dpi=100):
    for frame in range(round(DURATION * FPS)):
        draw_frame(frame / FPS)
        writer.grab_frame()
plt.close(fig)
print(f"Saved {OUTPUT}: {DURATION:.1f} seconds at {FPS} fps")
if display is not None:
    display(Video(OUTPUT, embed=True))
