"""Colab-ready 3D wall-climbing robot design and animated kinematic simulation.

Creates a six-second MP4 and a static STL of the robot in one pose.
The STL is a concept mesh, not a manufacturing-ready or load-validated design.
"""

import math
import shutil
import struct
from dataclasses import dataclass

import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

try:
    from IPython.display import Video, display
except ImportError:  # Colab includes IPython; local headless rendering need not.
    Video = display = None

# Concept dimensions are in meters. Change these to explore the motion.
DURATION = 6.0
FPS = 15
CLIMB_SPEED = 0.28
GAIT_PERIOD = 1.5
BODY_WIDTH = 0.66
BODY_HEIGHT = 0.72
PAD_RADIUS = 0.105
VIDEO_FILE = "wall_climbing_robot_3d.mp4"
STL_FILE = "wall_climbing_robot_3d_concept.stl"

if shutil.which("ffmpeg") is None:
    raise RuntimeError("FFmpeg is required. In Colab run: !apt-get -qq install ffmpeg")


@dataclass
class Mesh:
    vertices: np.ndarray
    triangles: np.ndarray
    color: str


def box(center, size, color):
    cx, cy, cz = center
    sx, sy, sz = np.asarray(size) / 2
    vertices = np.array([
        [cx-sx, cy-sy, cz-sz], [cx+sx, cy-sy, cz-sz],
        [cx+sx, cy+sy, cz-sz], [cx-sx, cy+sy, cz-sz],
        [cx-sx, cy-sy, cz+sz], [cx+sx, cy-sy, cz+sz],
        [cx+sx, cy+sy, cz+sz], [cx-sx, cy+sy, cz+sz]
    ], dtype=float)
    triangles = np.array([
        [0,2,1], [0,3,2], [4,5,6], [4,6,7],
        [0,4,7], [0,7,3], [1,2,6], [1,6,5],
        [0,1,5], [0,5,4], [3,7,6], [3,6,2]
    ], dtype=int)
    return Mesh(vertices, triangles, color)


def cylinder(start, end, radius, color, sides=12):
    """Triangulated solid tube between arbitrary 3D points."""
    start, end = np.asarray(start, float), np.asarray(end, float)
    axis = end - start
    axis /= np.linalg.norm(axis)
    reference = np.array([0., 1., 0.])
    if abs(axis @ reference) > 0.9:
        reference = np.array([1., 0., 0.])
    u = np.cross(axis, reference)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    angles = np.arange(sides) * 2 * np.pi / sides
    ring = radius * (np.cos(angles)[:, None] * u + np.sin(angles)[:, None] * v)
    vertices = np.vstack([start + ring, end + ring, start, end])
    faces = []
    for i in range(sides):
        j = (i + 1) % sides
        faces += [[i, j, sides+j], [i, sides+j, sides+i],
                  [2*sides, j, i], [2*sides+1, sides+i, sides+j]]
    return Mesh(vertices, np.asarray(faces, dtype=int), color)


def pad_pose(t, side, upper, offset):
    """A sealed pad is stationary in world coordinates during stance."""
    phase = (t / GAIT_PERIOD + offset) % 1.0
    grip_fraction = 0.72
    step = CLIMB_SPEED * GAIT_PERIOD
    base_y = 0.32 if upper else -0.32
    first_y = base_y + grip_fraction * step / 2
    if phase < grip_fraction:
        return side * 0.77, first_y - step * phase, 0.0, True
    q = (phase - grip_fraction) / (1 - grip_fraction)
    smooth = q*q*(3 - 2*q)
    x = side * (0.77 + 0.05 * math.sin(math.pi*q))
    y = first_y - grip_fraction*step + grip_fraction*step*smooth
    return x, y + 0.045*math.sin(math.pi*q), 0.13*math.sin(math.pi*q), False


def make_robot(t):
    """Return colored robot components and number of attached suction pads."""
    body_y = 0.78 + CLIMB_SPEED*t
    pieces = [
        box((0, body_y, 0.285), (0.73, 0.80, 0.15), "#13232d"),
        box((0, body_y, 0.410), (BODY_WIDTH, BODY_HEIGHT, 0.22), "#164153"),
        box((0, body_y-0.03, 0.535), (0.51, 0.48, 0.045), "#e4a84b"),
        box((0, body_y+0.235, 0.565), (0.35, 0.08, 0.045), "#2a5361"),
        cylinder((0, body_y+0.06, 0.56), (0, body_y+0.06, 0.615),
                 0.085, "#c3d9da"),
        cylinder((0, body_y+0.06, 0.615), (0, body_y+0.06, 0.630),
                 0.039, "#32bad3")
    ]
    for x in [-0.19, 0.19]:
        pieces.append(cylinder((x, body_y+0.255, 0.565),
                               (x, body_y+0.255, 0.585), 0.025, "#75efbb"))

    feet = [(-1, True, 0.0), (1, True, 0.5),
            (-1, False, 0.5), (1, False, 0.0)]
    gripped = 0
    for side, upper, offset in feet:
        x, local_y, lift, sealed = pad_pose(t, side, upper, offset)
        y = body_y + local_y
        gripped += int(sealed)
        cup_z = 0.020 + lift
        shoulder = np.array([side*0.29, body_y+(0.23 if upper else -0.23), 0.36])
        elbow = np.array([side*0.54,
                          (shoulder[1] + y)/2 + (0.055 if upper else -0.055),
                          0.24 + lift/2])
        target = np.array([x, y, cup_z+0.13])
        pieces.extend([
            cylinder(shoulder, elbow, 0.043, "#a3bec2"),
            cylinder(elbow, target, 0.034, "#c3d1d0"),
            cylinder(elbow-[0,0,0.034], elbow+[0,0,0.034],
                     0.064, "#efb556"),
            cylinder((x, y, cup_z), (x, y, cup_z+0.075),
                     PAD_RADIUS, "#2f755e" if sealed else "#b77841"),
            cylinder((x, y, cup_z+0.074), (x, y, cup_z+0.105),
                     PAD_RADIUS+0.021, "#76e8b3" if sealed else "#f3b778"),
            cylinder((x, y, cup_z+0.105), target,
                     0.041, "#566d74")
        ])
    return pieces, gripped


def triangles_and_colors(meshes):
    triangles = np.concatenate([m.vertices[m.triangles] for m in meshes])
    color = np.concatenate([
        np.tile(mcolors.to_rgb(m.color), (len(m.triangles), 1)) for m in meshes
    ])
    normals = np.cross(triangles[:,1]-triangles[:,0],
                       triangles[:,2]-triangles[:,0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1)[:,None], 1e-12)
    light = np.array([-0.4, 0.65, 0.70])
    light /= np.linalg.norm(light)
    shade = np.clip(0.70 + 0.24*(normals @ light), 0.43, 1.0)
    return triangles, np.clip(color * shade[:,None], 0, 1)


def save_binary_stl(path, meshes):
    """Write the robot upright; STL coordinates are millimeters."""
    triangles, _ = triangles_and_colors(meshes)
    # Model coordinates (x, height y, depth z) become CAD (x, depth y, up z).
    # Reversing each face restores outward normals after swapping axes.
    triangles = triangles[:, [0,2,1], :][:, :, [0,2,1]] * 1000.0
    triangles[:,:,1] -= triangles[:,:,1].min()  # wall contact near Y=0
    triangles[:,:,2] -= triangles[:,:,2].min()  # lowest pad at Z=0
    normals = np.cross(triangles[:,1]-triangles[:,0],
                       triangles[:,2]-triangles[:,0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1)[:,None], 1e-12)
    with open(path, "wb") as stl:
        stl.write(b"WC-01 conceptual robot assembly, units millimeters".ljust(80, b" "))
        stl.write(struct.pack("<I", len(triangles)))
        for normal, tri in zip(normals, triangles):
            stl.write(struct.pack("<12fH", *normal, *tri.reshape(-1), 0))


wall = box((0, 1.58, -0.085), (2.12, 3.16, 0.17), "#304550")
wall_faces, wall_colors = triangles_and_colors([wall])
brick_segments = []
for row, y in enumerate(np.arange(0.06, 3.16, 0.29)):
    brick_segments.append([[-1.06,y,0.006], [1.06,y,0.006]])
    stagger = 0.23 if row % 2 else 0.0
    for x in np.arange(-1.06 + stagger, 1.07, 0.46):
        brick_segments.append([[x,y,0.006], [x,min(y+0.29,3.16),0.006]])


def render_frame(fig, ax, t):
    ax.cla()
    ax.set_facecolor("#08141d")
    ax.set_xlim(-1.18, 1.18)
    ax.set_ylim(-0.29, 0.90)
    ax.set_zlim(-0.05, 3.21)
    ax.set_box_aspect((2.36, 1.19, 3.26), zoom=1.07)
    ax.set_proj_type("persp", focal_length=1.7)
    ax.view_init(elev=11, azim=72 + 24*t/DURATION)
    ax.set_axis_off()

    # Plotting coordinates are (horizontal x, depth z, vertical y).
    ax.add_collection3d(Poly3DCollection(wall_faces[:,:,[0,2,1]], facecolors=wall_colors,
                                          linewidths=0.0, antialiased=False,
                                          zorder=1))
    ax.add_collection3d(Line3DCollection(np.asarray(brick_segments)[:,:,[0,2,1]],
                                          colors="#4b616b",
                                          linewidths=0.9, zorder=2))
    robot, gripped = make_robot(t)
    faces, colors = triangles_and_colors(robot)
    ax.add_collection3d(Poly3DCollection(faces[:,:,[0,2,1]], facecolors=colors,
                                          linewidths=0.0, antialiased=False,
                                          zorder=3))
    return gripped


def main():
    robot_snapshot, _ = make_robot(2.5)  # all four pads sealed
    save_binary_stl(STL_FILE, robot_snapshot)

    fig = plt.figure(figsize=(10, 7), dpi=100, facecolor="#08141d")
    ax = fig.add_subplot(111, projection="3d", computed_zorder=False)
    ax.set_position([0.06, 0.09, 0.88, 0.80])
    fig.text(0.5, 0.957, "WC-01  |  WALL-CLIMBING ROBOT", ha="center",
             va="top", fontsize=19, weight="bold", color="#eff7f5")
    fig.text(0.5, 0.912, "3D CONCEPT MODEL   •   FOUR SUCTION PADS   •   ALTERNATING GAIT",
             ha="center", va="top", fontsize=9, color="#8faeb6")
    status = fig.text(0.5, 0.044, "", ha="center", va="bottom",
                      fontsize=12, color="#84e9bf")
    fig.text(0.5, 0.015, "Illustrative motion; suction, friction and motor forces are not simulated",
             ha="center", fontsize=8, color="#8da5ac")

    writer = FFMpegWriter(fps=FPS, bitrate=1800,
                          metadata={"title": "WC-01 3D wall-climbing robot"})
    with writer.saving(fig, VIDEO_FILE, dpi=100):
        for frame in range(round(DURATION*FPS)):
            t = frame/FPS
            grip_count = render_frame(fig, ax, t)
            status.set_text(f"HEIGHT  {CLIMB_SPEED*t:.2f} m      •      "
                            f"SEALED  {grip_count}/4      •      TIME  {t:.1f} s")
            writer.grab_frame()
    plt.close(fig)
    print(f"Created {VIDEO_FILE} ({DURATION:.0f} s, {FPS} fps) and {STL_FILE}")
    if display is not None:
        display(Video(VIDEO_FILE, embed=True))


if __name__ == "__main__":
    main()
